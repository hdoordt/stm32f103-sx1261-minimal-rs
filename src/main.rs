#![no_std]
#![no_main]

use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use stm32::SPI1;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::gpio::{
    gpioa::{PA0, PA5, PA6, PA7, PA8},
    gpiob::{PB10, PB5},
    Alternate, Floating, Input, Output, PushPull,
    State::High,
};
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::spi::{Mode as SpiMode, Phase, Polarity, Spi, Spi1NoRemap};
use stm32f1xx_hal::stm32;

type Spi1 = Spi<
    SPI1,
    Spi1NoRemap,
    (
        PA5<Alternate<PushPull>>,
        PA6<Input<Floating>>,
        PA7<Alternate<PushPull>>,
    ),
>;

type NssPin = PA8<Output<PushPull>>;
type BusyPin = PB5<Input<Floating>>;
type NrstPin = PA0<Output<PushPull>>;
type Dio1Pin = PB10<Input<Floating>>;

#[entry]
fn main() -> ! {
    let peripherals = stm32::Peripherals::take().unwrap();
    let core_peripherals = stm32::CorePeripherals::take().unwrap();

    let mut rcc = peripherals.RCC.constrain();
    let mut flash = peripherals.FLASH.constrain();
    let mut afio = peripherals.AFIO.constrain(&mut rcc.apb2);

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = peripherals.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = peripherals.GPIOB.split(&mut rcc.apb2);

    // Init LED on pin D2
    let mut led_pin = gpioa
        .pa10
        .into_push_pull_output_with_state(&mut gpioa.crh, High);

    // Init SPI1
    let spi1_sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let spi1_miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let spi1_mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let spi1_mode = SpiMode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi1_freq = 100.khz();

    let spi1_pins = (
        spi1_sck,  // D13
        spi1_miso, // D12
        spi1_mosi, // D11
    );

    let spi1 = Spi::spi1(
        peripherals.SPI1,
        spi1_pins,
        &mut afio.mapr,
        spi1_mode,
        spi1_freq,
        clocks,
        &mut rcc.apb2,
    );

    // Init SX1261 pins

    let lora_nreset = gpioa
        .pa0
        .into_push_pull_output_with_state(&mut gpioa.crl, High);

    let lora_nss = gpioa
        .pa8
        .into_push_pull_output_with_state(&mut gpioa.crh, High);

    let lora_busy = gpiob.pb5.into_floating_input(&mut gpiob.crl);

    let lora_dio1 = gpiob.pb10.into_floating_input(&mut gpiob.crh);
    // D8
    let _lora_ant = gpioa
        .pa9
        .into_push_pull_output_with_state(&mut gpioa.crh, High);

    let delay = Delay::new(core_peripherals.SYST, clocks);

    let mut sx = LoRa {
        nss: lora_nss,       // D7
        busy: lora_busy,     // D4
        nreset: lora_nreset, // A0
        dio1: lora_dio1,     // D6
        delay,
        spi: spi1,
    };

    let message_payload = b"Hello, LoRa world!";

    // Reset the device
    sx.nreset.set_low().unwrap();
    sx.delay.delay_us(200u8);
    sx.nreset.set_high().unwrap();

    // 14.2 Circuit Configuration for Basic Tx Operation

    // 1. If not in STDBY_RC mode, then go to this mode with the command SetStandby(...)
    // Standby mode: STDBY_RC
    sx.spi_write(&[0x80, 0x00]).unwrap();

    // 2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
    // Packet type = LoRa
    sx.spi_write(&[0x8A, 0x01]).unwrap();

    // 3. Define the RF frequency with the command SetRfFrequency(...)
    // 13.4.1.: RFfrequency = (RFfreq * Fxtal) / 2^25 = 868M
    // RFfreq = RFfrequency * (2^25/32,000,000)
    // = 868M * (2^25/32,000,000)
    // = 910163968
    let rf_frequecy: u32 = 910163968;
    sx.spi_cmd(|spi| {
        spi.write(&[0x86])
            .and_then(|_| spi.write(&rf_frequecy.to_be_bytes()))
            .unwrap()
    })
    .unwrap();

    // Calibrate
    sx.spi_write(&[0x89, 0x7F]).unwrap();

    // Calibrate Image
    sx.spi_write(&[0x98, 0xD7, 0xDB]).unwrap();

    // 4. Define the Power Amplifier configuration with the command SetPaConfig(...)
    // Duty cycle: 0x04
    // hp max: 0x00
    // device sel: sx1261 (0x01)
    sx.spi_write(&[0x95, 0x04, 0x00, 0x01, 0x01]).unwrap();

    // 5. Define output power and ramping time with the command SetTxParams(...)
    // Power: 14 dBm
    // Ramp time: 200 us
    sx.spi_write(&[0x8E, 0x0E, 0x04]).unwrap();

    // 6. Define where the data payload will be stored with the command SetBufferBaseAddress(...)
    // TX buffer base address: 0x00
    // RX buffer base address: 0x00
    sx.spi_write(&[0x8F, 0x00, 0x00]).unwrap();

    // 7. Send the payload to the data buffer with the command WriteBuffer(...)
    sx.spi_cmd(|spi| {
        spi.write(&[0x0E, 0x00])
            .and_then(|_| spi.write(message_payload))
            .unwrap()
    })
    .unwrap();

    // 8. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...) 1
    // Spread factor: SF7
    // Bandwidth: LORA_BW_125
    // Coding rate: LORA_CR_4_5
    // Low data rate optimize: OFF
    sx.spi_write(&[0x8B, 0x07, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        .unwrap();

    // 9. Define the frame format to be used with the command SetPacketParams(...) 2
    // Preamble length: 8
    // Header type: Variable length packet (explicit header)
    // Payload length: length of `message_payload`
    // CRC type: ON
    // Invert IQ: Standard IQ setup
    let preamble_length = 8u16.to_be_bytes();
    sx.spi_cmd(|spi| {
        spi.write(&[0x8C])
            .and_then(|_| spi.write(&preamble_length)) // 1 and 2: preamble length
            .and_then(|_| {
                spi.write(&[
                    0x00, // 3: header type (explicit)
                    message_payload.len() as u8, // 4 payload length
                    0x01, // 5: crc type (on)
                    0x00, // 6: invert IQ (standard)
                    0x00, // 7: RFU
                    0x00, // 8: RFU
                ])
            })
            .unwrap()
    })
    .unwrap();

    // 10. Configure DIO and IRQ: use the command SetDioIrqParams(...) to select TxDone IRQ and map this IRQ to a DIO (DIO1,
    // DIO2 or DIO3)
    // IRQ Mask: TxDone|Timeout =
    // DIO1 Mask: TxDone|Timeout
    // DIO2 Mask: None
    // DIO3 Mask: None
    sx.spi_write(&[0x08, 0x02, 0x01, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00])
        .unwrap();

    // SetDIO2AsRfSwitchCtrl
    sx.spi_write(&[0x9D, 0x01]).unwrap();

    // 11. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access
    // Assumption: sx1276 sync word = 0x12
    // see https://www.thethingsnetwork.org/forum/t/should-private-lorawan-networks-use-a-different-sync-word/34496/15
    // Sync word = 0x1424
    sx.spi_write(&[0x0D, 0x07, 0x40, 0x14, 0x24]).unwrap();

    sx.delay.delay_ms(500u16);
    led_pin.set_low().unwrap();
    // 12. Set the circuit in transmitter mode to start transmission with the command SetTx(). Use the parameter to enable Timeout
    // Timeout: 0 (disabled)
    sx.spi_write(&[0x83, 0x00, 0x00, 0x00]).unwrap();

    // 13. Wait for the IRQ TxDone or Timeout: once the packet has been sent the chip goes automatically to STDBY_RC mode
    sx.wait_on_busy();
    // Wait until dio1 becomes high
    while sx.dio1.is_low().unwrap() {
        cortex_m::asm::nop();
    }

    // 14. Clear the IRQ TxDone flag
    // Clear all flags
    sx.spi_write(&[0x02, 0xFF, 0xFF]).unwrap();

    sx.delay.delay_ms(500u16);
    led_pin.set_high().unwrap();
    loop {}
}

/// Wrapper around every peripheral needed to communicate with the sx1261
struct LoRa {
    nss: NssPin,
    busy: BusyPin,
    nreset: NrstPin,
    dio1: Dio1Pin,
    delay: Delay,
    spi: Spi1,
}

impl LoRa {
    /// Handles pulling nss low and delaying before actually doing any spi transaction.
    /// It also blocks while the busy line is high.
    fn spi_cmd<F>(&mut self, mut cmd: F) -> core::result::Result<(), core::convert::Infallible>
    where
        F: FnMut(&mut Spi1),
    {
        self.wait_on_busy();
        self.nss.set_low()?;
        // Table 8-1: Data sheet specifies a minumum delay of 32ns between falling edge of nss and sck setup,
        // though embedded_hal provides no trait for delaying in nanosecond resolution.
        self.delay.delay_us(1u8);
        cmd(&mut self.spi);
        self.nss.set_high()
    }

    /// Write a message over spi. Calls spi_cmd internally
    fn spi_write(&mut self, bytes: &[u8]) -> core::result::Result<(), core::convert::Infallible> {
        self.spi_cmd(
            |spi| spi.write(bytes).unwrap(), // Panic if anything goes wrong
        )
    }

    /// Transfer data over spi. Calls spi_cmd internally
    fn spi_transfer(
        &mut self,
        bytes: &mut [u8],
    ) -> core::result::Result<(), core::convert::Infallible> {
        self.spi_cmd(
            |spi| spi.transfer(bytes).map(|_| {}).unwrap(), // Panic if anything goes wrong
        )
    }

    /// Wait until the busy line is low
    fn wait_on_busy(&mut self) {
        // 8.3.1: The max value for T SW from NSS rising edge to the BUSY rising edge is, in all cases, 600 ns
        self.delay.delay_us(1u8);
        // Busy wait for simplicity
        while self.busy.is_high().unwrap() {
            cortex_m::asm::nop();
        }
    }
}
