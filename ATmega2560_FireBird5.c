// =============================================================
//  main.c  –  Firebird V ATmega2560
//             PWM Differential Drive Receiver + Dual Sensor Telemetry TX
//
//  Architecture:
//    ESP-01  –[UART3 115200]–>  ATmega2560  (3-byte PWM command IN)
//    ATmega2560  –[UART3 115200]–>  ESP-01  (telemetry OUT)
//    ATmega2560  –[UART2 115200]–>  Debug monitor (telemetry + debug OUT)
//
//  Command packet format (3 bytes from ESP-01):
//    byte[0] = direction (0x06=FWD, 0x09=BWD, 0x00=STOP)
//    byte[1] = left  motor PWM (0-255) ? OCR5AL
//    byte[2] = right motor PWM (0-255) ? OCR5BL
//
//  Telemetry packet format (sent every 200ms via Timer3):
//    $BAT:x.xx,WL:aaa:bbb:ccc,IR:aaa:bbb:ccc,DIST:dddd,ENC:lllll:rrrrr,CMD:X\n
//
//  Timers used:
//    Timer1 – 500ms watchdog (auto-stop if ESP-01 goes silent)
//    Timer3 – 200ms telemetry heartbeat
//    Timer5 – Fast PWM 8-bit motor speed control
//
//  Atmel Studio settings:
//    Microcontroller : atmega2560
//    Frequency       : 14745600
//    Optimization    : -O0
// =============================================================

#define F_CPU 14745600UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// ?? Baud: 14745600 / (16 * 115200) - 1 = 7  ? 0% error ??????
#define BAUD     115200UL
#define UBRR_VAL ((F_CPU / (16UL * BAUD)) - 1)   // = 7

// ?? Timer1 – 500ms watchdog ???????????????????????????????????
#define WATCHDOG_TICKS 7200U

// ?? Timer3 – 200ms telemetry ??????????????????????????????????
#define TELEMETRY_TICKS 2880U

// ?? 3-byte command receive buffer ????????????????????????????
#define CMD_PACKET_LEN 3
volatile uint8_t g_cmd_buf[CMD_PACKET_LEN];
volatile uint8_t g_cmd_idx       = 0;
volatile uint8_t g_new_command   = 0;

// ?? Decoded motor state (set from ISR, applied in main) ??????
volatile uint8_t g_dir_byte      = 0x00;
volatile uint8_t g_left_pwm      = 0;
volatile uint8_t g_right_pwm     = 0;

// ?? For telemetry CMD field ???????????????????????????????????
volatile char    g_command       = 'S';   // 'F', 'B', or 'S'

// ?? Timeout / telemetry flags ?????????????????????????????????
volatile uint8_t g_timeout_flag   = 0;
volatile uint8_t g_send_telemetry = 0;

// ?? Encoder counts ????????????????????????????????????????????
volatile unsigned long int ShaftCountLeft  = 0;
volatile unsigned long int ShaftCountRight = 0;

// ?? Hardware pin configs ??????????????????????????????????????
void motion_pin_config(void)
{
	DDRA  = 0xFF;
	PORTA = 0x00;
	DDRL  = 0xFF;
	PORTL = 0xFF;
}

void left_encoder_pin_config(void)
{
	DDRE  = DDRE & 0xEF;
	PORTE = PORTE | 0x10;
}

void right_encoder_pin_config(void)
{
	DDRE  = DDRE & 0xDF;
	PORTE = PORTE | 0x20;
}

void adc_pin_config(void)
{
	DDRF  = 0x00; PORTF = 0x00;
	DDRK  = 0x00; PORTK = 0x00;
}

// ?? ADC ???????????????????????????????????????????????????????
void adc_init(void)
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;
	ADMUX  = 0x20;
	ACSR   = 0x80;
	ADCSRA = 0x86;
}

unsigned char ADC_Conversion(unsigned char ch)
{
	unsigned char result;
	if (ch > 7) { ADCSRB = 0x08; }
	ch     = ch & 0x07;
	ADMUX  = 0x20 | ch;
	ADCSRA = ADCSRA | 0x40;
	while ((ADCSRA & 0x10) == 0);
	result = ADCH;
	ADCSRA = ADCSRA | 0x10;
	ADCSRB = 0x00;
	return result;
}

unsigned int battery_voltage_x100(void)
{
	unsigned char raw = ADC_Conversion(0);
	float v = ((raw * 100.0f) * 0.07902f) + 0.7f;
	return (unsigned int)(v * 100.0f + 0.5f);
}

unsigned int sharp_distance_mm(unsigned char adc_val)
{
	if (adc_val == 0) return 800;
	float d = 10.0f * (2799.6f * (1.0f / powf((float)adc_val, 1.1546f)));
	unsigned int di = (unsigned int)d;
	return (di > 800) ? 800 : di;
}

// ?? UART2 (wired debug monitor) ???????????????????????????????
void uart2_init(void)
{
	UBRR2H = (uint8_t)(UBRR_VAL >> 8);
	UBRR2L = (uint8_t)(UBRR_VAL);
	UCSR2A = 0x00;
	UCSR2B = (1 << TXEN2);
	UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);
}

void uart2_send_byte(unsigned char c)
{
	while (!(UCSR2A & (1 << UDRE2)));
	UDR2 = c;
}

void uart2_send_string(const char *s)
{
	while (*s) uart2_send_byte((unsigned char)*s++);
}

// ?? UART3 (ESP-01 link) ???????????????????????????????????????
void uart3_init(void)
{
	UBRR3H = (uint8_t)(UBRR_VAL >> 8);
	UBRR3L = (uint8_t)(UBRR_VAL);
	UCSR3A = 0x00;
	UCSR3B = (1 << RXEN3) | (1 << TXEN3) | (1 << RXCIE3);
	UCSR3C = (1 << UCSZ31) | (1 << UCSZ30);
}

void uart3_send_byte(unsigned char c)
{
	while (!(UCSR3A & (1 << UDRE3)));
	UDR3 = c;
}

void uart3_send_string(const char *s)
{
	while (*s) uart3_send_byte((unsigned char)*s++);
}

// ?? Timer1 – 500ms watchdog ???????????????????????????????????
void timer1_init(void)
{
	TCCR1A = 0x00;
	TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);
	OCR1A  = WATCHDOG_TICKS;
	TIMSK1 = (1 << OCIE1A);
}

// ?? Timer3 – 200ms telemetry ??????????????????????????????????
void timer3_init(void)
{
	TCCR3A = 0x00;
	TCCR3B = (1 << WGM32) | (1 << CS32) | (1 << CS30);
	OCR3A  = TELEMETRY_TICKS;
	TIMSK3 = (1 << OCIE3A);
}

// ?? Timer5 – Fast PWM 8-bit motor speed ??????????????????????
//  TCCR5A = 0xA9 ? COM5A1, COM5B1 (non-inverting), WGM50 (Fast PWM 8-bit)
//  TCCR5B = 0x0B ? WGM52, CS51|CS50 (prescaler /64)
//  OCR5AL ? left  motor (OC5A, PORTL.3)
//  OCR5BL ? right motor (OC5B, PORTL.4)
void timer5_init(void)
{
	TCCR5A = 0xA9;
	TCCR5B = 0x0B;
	OCR5AL = 0xFF;   // start at full speed; motion_set(0x00) holds motors stopped
	OCR5BL = 0xFF;
}

// ?? ISRs ??????????????????????????????????????????????????????
ISR(USART3_RX_vect)
{
	uint8_t byte = UDR3;
	g_cmd_buf[g_cmd_idx++] = byte;

	if (g_cmd_idx == CMD_PACKET_LEN)
	{
		g_cmd_idx = 0;

		uint8_t dir  = g_cmd_buf[0];
		uint8_t lp   = g_cmd_buf[1];
		uint8_t rp   = g_cmd_buf[2];

		// Validate direction byte
		if (dir == 0x06 || dir == 0x09 || dir == 0x00)
		{
			g_dir_byte   = dir;
			g_left_pwm   = lp;
			g_right_pwm  = rp;
			g_command    = (dir == 0x06) ? 'F' : (dir == 0x09) ? 'B' : 'S';
			g_new_command = 1;
			TCNT1         = 0;
			g_timeout_flag = 0;
		}
		else
		{
			// Malformed direction byte — log via UART2
			char dbg[40];
			snprintf(dbg, sizeof(dbg), "DBG: BAD_DIR 0x%02X L=%u R=%u\n", dir, lp, rp);
			uart2_send_string(dbg);
		}
	}
	else if (g_cmd_idx > CMD_PACKET_LEN)
	{
		// Overrun guard — reset accumulator
		g_cmd_idx = 0;
		uart2_send_string("DBG: PKT_OVERRUN\n");
	}
}

ISR(TIMER1_COMPA_vect) { g_timeout_flag   = 1; }
ISR(TIMER3_COMPA_vect) { g_send_telemetry = 1; }
ISR(INT4_vect)         { ShaftCountLeft++;      }
ISR(INT5_vect)         { ShaftCountRight++;     }

void encoder_interrupt_init(void)
{
	EICRB = EICRB | 0x0A;
	EIMSK = EIMSK | 0x30;
}

// ?? Motor direction control ???????????????????????????????????
void motion_set(unsigned char dir)
{
	unsigned char pa = PORTA & 0xF0;
	PORTA = pa | (dir & 0x0F);
}

void stop(void) { motion_set(0x00); OCR5AL = 0; OCR5BL = 0; }

// ?? Apply decoded 3-byte command ?????????????????????????????
void apply_pwm_command(uint8_t dir, uint8_t lp, uint8_t rp)
{
	motion_set(dir);
	OCR5AL = lp;
	OCR5BL = rp;

	// Debug log via UART2
	const char* label = (dir == 0x06) ? "FWD"
	: (dir == 0x09) ? "BWD"
	:                 "STP";
	char dbg[48];
	snprintf(dbg, sizeof(dbg), "DBG: DIR=%s L=%u R=%u\n", label, lp, rp);
	uart2_send_string(dbg);
}

// ?? Telemetry packet ??????????????????????????????????????????
void send_telemetry_packet(void)
{
	char buf[128];

	unsigned int  bat   = battery_voltage_x100();
	unsigned char wl1   = ADC_Conversion(3);
	unsigned char wl2   = ADC_Conversion(2);
	unsigned char wl3   = ADC_Conversion(1);
	unsigned char ir2   = ADC_Conversion(5);
	unsigned char ir3   = ADC_Conversion(6);
	unsigned char ir4   = ADC_Conversion(7);
	unsigned char sharp = ADC_Conversion(11);
	unsigned int  dist  = sharp_distance_mm(sharp);

	cli();
	unsigned long encL = ShaftCountLeft;
	unsigned long encR = ShaftCountRight;
	char          cmd  = g_command;
	sei();

	snprintf(buf, sizeof(buf),
	"$BAT:%u.%02u,WL:%u:%u:%u,IR:%u:%u:%u,DIST:%u,ENC:%lu:%lu,CMD:%c\n",
	bat / 100, bat % 100,
	wl1, wl2, wl3,
	ir2, ir3, ir4,
	dist,
	encL, encR,
	cmd
	);

	uart3_send_string(buf);
	uart2_send_string(buf);
}

// ?? Device initialization ?????????????????????????????????????
void init_devices(void)
{
	cli();
	motion_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
	adc_pin_config();
	adc_init();
	uart2_init();
	uart3_init();
	timer1_init();
	timer3_init();
	timer5_init();
	encoder_interrupt_init();
	sei();
}

// ?? Main loop ?????????????????????????????????????????????????
int main(void)
{
	init_devices();
	stop();

	while (1)
	{
		// ?? Watchdog timeout: stop motors ?????????????????????
		if (g_timeout_flag)
		{
			g_timeout_flag = 0;
			g_dir_byte     = 0x00;
			g_left_pwm     = 0;
			g_right_pwm    = 0;
			g_command      = 'S';
			cli();
			ShaftCountLeft  = 0;
			ShaftCountRight = 0;
			sei();
			stop();
			uart2_send_string("DBG: WATCHDOG_TIMEOUT - motors stopped\n");
		}

		// ?? New 3-byte command received ???????????????????????
		if (g_new_command)
		{
			cli();
			uint8_t dir = g_dir_byte;
			uint8_t lp  = g_left_pwm;
			uint8_t rp  = g_right_pwm;
			g_new_command = 0;
			sei();
			apply_pwm_command(dir, lp, rp);
		}

		// ?? 200ms telemetry heartbeat ?????????????????????????
		if (g_send_telemetry)
		{
			g_send_telemetry = 0;
			send_telemetry_packet();
		}
	}
}