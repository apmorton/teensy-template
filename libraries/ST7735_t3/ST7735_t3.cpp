/*************************************************** 
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "ST7735_t3.h"
#include "ST7789_t3.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

#ifdef ENABLE_ST77XX_FRAMEBUFFER
//#define DEBUG_ASYNC_UPDATE
//#define DEBUG_ASYNC_LEDS
#ifdef DEBUG_ASYNC_LEDS
  #define DEBUG_PIN_1 0
  #define DEBUG_PIN_2 1
  #define DEBUG_PIN_3 2
#endif

volatile short _dma_dummy_rx;

ST7735_t3 *ST7735_t3::_dmaActiveDisplay[3] = {0, 0, 0};

#if defined(__MK66FX1M0__) 
 DMASetting   ST7735_t3::_dmasettings[3][4];
#endif

#if defined(__IMXRT1062__)  // Teensy 4.x
// On T4 Setup the buffers to be used one per SPI buss... 
// This way we make sure it is hopefully in uncached memory
ST7735DMA_Data ST7735_t3::_dma_data[3];   // one structure for each SPI buss... 
#endif

#endif

// Constructor when using software SPI.  All output pins are configurable.
ST7735_t3::ST7735_t3(uint8_t cs, uint8_t rs, uint8_t sid, uint8_t sclk, uint8_t rst)
{
	_cs   = cs;
	_rs   = rs;
	_sid  = sid;
	_sclk = sclk;
	_rst  = rst;
	_rot = 0xff;
	hwSPI = false;
	#ifdef ENABLE_ST77XX_FRAMEBUFFER
    _pfbtft = NULL;	
    _use_fbtft = 0;						// Are we in frame buffer mode?
	_we_allocated_buffer = NULL;
	_dma_state = 0;
    #endif
	_screenHeight = ST7735_TFTHEIGHT_160;
	_screenWidth = ST7735_TFTWIDTH;	
	
	_width = _screenWidth;
	_height = _screenHeight;
	
	cursor_y  = cursor_x    = 0;
	textsize_x  = 1;
	textsize_y  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	gfxFont   = NULL;
	setClipRect();
	setOrigin();
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ST7735_t3::ST7735_t3(uint8_t cs, uint8_t rs, uint8_t rst) 
{
	_cs   = cs;
	_rs   = rs;
	_rst  = rst;
	_rot = 0xff;
	hwSPI = true;
	_sid  = _sclk = (uint8_t)-1;
	#ifdef ENABLE_ST77XX_FRAMEBUFFER
    _pfbtft = NULL;	
    _use_fbtft = 0;						// Are we in frame buffer mode?
	_we_allocated_buffer = NULL;
	_dma_state = 0;
    #endif
	_screenHeight = ST7735_TFTHEIGHT_160;
	_screenWidth = ST7735_TFTWIDTH;

	cursor_y  = cursor_x    = 0;
	textsize_x  = 1;
	textsize_y  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	gfxFont   = NULL;
	setClipRect();
	setOrigin();
}



/***************************************************************/
/*     Teensy 3.0, 3.1, 3.2, 3.5, 3.6                          */
/***************************************************************/
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

inline void ST7735_t3::waitTransmitComplete(void)  {
    uint32_t tmp __attribute__((unused));
    while (!(_pkinetisk_spi->SR & SPI_SR_TCF)) ; // wait until final output done
    tmp = _pkinetisk_spi->POPR;                  // drain the final RX FIFO word
}

inline void ST7735_t3::waitTransmitComplete(uint32_t mcr) {
    uint32_t tmp __attribute__((unused));
    while (1) {
        uint32_t sr = _pkinetisk_spi->SR;
        if (sr & SPI_SR_EOQF) break;  // wait for last transmit
        if (sr &  0xF0) tmp = _pkinetisk_spi->POPR;
    }
    _pkinetisk_spi->SR = SPI_SR_EOQF;
    _pkinetisk_spi->MCR = mcr;
    while (_pkinetisk_spi->SR & 0xF0) {
        tmp = _pkinetisk_spi->POPR;
    }
}

inline void ST7735_t3::spiwrite(uint8_t c)
{
	// pass 1 if we actually are setup to with MOSI and SCLK on hardware SPI use it...
	if (_pspi) {
		_pspi->transfer(c);
		return;
	}

	for (uint8_t bit = 0x80; bit; bit >>= 1) {
		*datapin = ((c & bit) ? 1 : 0);
		*clkpin = 1;
		*clkpin = 0;
	}
}

inline void ST7735_t3::spiwrite16(uint16_t d)
{
	// pass 1 if we actually are setup to with MOSI and SCLK on hardware SPI use it...
	if (_pspi) {
		_pspi->transfer16(d);
		return;
	}
	spiwrite(d >> 8);
	spiwrite(d);
}

void ST7735_t3::writecommand(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 0;
		spiwrite(c);
	}
}

void ST7735_t3::writecommand_last(uint8_t c) {
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 0;
		spiwrite(c);
	}
}

void ST7735_t3::writedata(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 1;
		spiwrite(c);
	}
}

void ST7735_t3::writedata_last(uint8_t c)
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		spiwrite(c);
	}
}

void ST7735_t3::writedata16(uint16_t d)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 1;
		spiwrite16(d);
	}
}


void ST7735_t3::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		spiwrite16(d);
	}
}


#define CTAR_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_16MHz   (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
#define CTAR_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))

void ST7735_t3::setBitrate(uint32_t n)
{
	if (n >= 24000000) {
		ctar = CTAR_24MHz;
	} else if (n >= 16000000) {
		ctar = CTAR_16MHz;
	} else if (n >= 12000000) {
		ctar = CTAR_12MHz;
	} else if (n >= 8000000) {
		ctar = CTAR_8MHz;
	} else if (n >= 6000000) {
		ctar = CTAR_6MHz;
	} else {
		ctar = CTAR_4MHz;
	}
	SIM_SCGC6 |= SIM_SCGC6_SPI0;
	_pkinetisk_spi->MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
	_pkinetisk_spi->CTAR0 = ctar | SPI_CTAR_FMSZ(7);
	_pkinetisk_spi->CTAR1 = ctar | SPI_CTAR_FMSZ(15);
	_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
}


/***************************************************************/
/*     Teensy 4.                                               */
/***************************************************************/
#elif defined(__IMXRT1062__)  // Teensy 4.x
inline void ST7735_t3::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (_pspi) {
		_pspi->transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) DIRECT_WRITE_HIGH(_mosiport, _mosipinmask);
			else        DIRECT_WRITE_LOW(_mosiport, _mosipinmask);
			DIRECT_WRITE_HIGH(_sckport, _sckpinmask);
			asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
			DIRECT_WRITE_LOW(_sckport, _sckpinmask);
		}
	}
}

void ST7735_t3::writecommand(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) /*| LPSPI_TCR_CONT*/);
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitFifoNotFull();
	} else {
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		spiwrite(c);
	}
}

void ST7735_t3::writecommand_last(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		spiwrite(c);
	}

}

void ST7735_t3::writedata(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(c);
	}
} 

void ST7735_t3::writedata_last(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(c);
	}
} 


void ST7735_t3::writedata16(uint16_t d)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
		_pimxrt_spi->TDR = d;
		_pending_rx_count++;	//
		waitFifoNotFull();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(d >> 8);
		spiwrite(d);
	}
} 

void ST7735_t3::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15));
		_pimxrt_spi->TDR = d;
//		_pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(d >> 8);
		spiwrite(d);
	}
} 

void ST7735_t3::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}


/***************************************************************/
/*     Teensy LC                                               */
/***************************************************************/
#elif defined(__MKL26Z64__)

void ST7735_t3::waitTransmitComplete()  {
	if(!_pkinetisl_spi) return; // Software SPI don't do anything
	while (_data_sent_not_completed) {
		uint16_t timeout_count = 0xff; // hopefully enough 
		while (!(_pkinetisl_spi->S & SPI_S_SPRF) && timeout_count--) ; // wait 
		uint8_t d __attribute__((unused));
		d = _pkinetisl_spi->DL;
		d = _pkinetisl_spi->DH;
		_data_sent_not_completed--; // We hopefully received our data...
	}
}

void ST7735_t3::spiwrite16(uint16_t data)  {
	if (_pkinetisl_spi) {
		if (!(_pkinetisl_spi->C2 & SPI_C2_SPIMODE)) {
			// Wait to change modes until any pending output has been done.
			waitTransmitComplete();
			_pkinetisl_spi->C2 = SPI_C2_SPIMODE; // make sure 8 bit mode.
		}
		uint8_t s;
		do {
			s = _pkinetisl_spi->S;
			 // wait if output buffer busy.
			// Clear out buffer if there is something there...
			if  ((s & SPI_S_SPRF)) {
				uint8_t d __attribute__((unused));
				d = _pkinetisl_spi->DL;
				d = _pkinetisl_spi->DH;
				_data_sent_not_completed--; 	// let system know we sent something	
			}

		} while (!(s & SPI_S_SPTEF) || (s & SPI_S_SPRF));

		_pkinetisl_spi->DL = data; 		// output low byte
		_pkinetisl_spi->DH = data >> 8; // output high byte
		_data_sent_not_completed++; 	// let system know we sent something	
	} else {
		// call bitbang functions	
		spiwrite(data >> 8);
		spiwrite(data);
	}
}

inline void ST7735_t3::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (_pkinetisl_spi) {
		if (_pkinetisl_spi->C2 & SPI_C2_SPIMODE) {
			// Wait to change modes until any pending output has been done.
			waitTransmitComplete();
			_pkinetisl_spi->C2 = 0; // make sure 8 bit mode.
		}
		while (!(_pkinetisl_spi->S & SPI_S_SPTEF)) ; // wait if output buffer busy.
		// Clear out buffer if there is something there...
		if  ((_pkinetisl_spi->S & SPI_S_SPRF)) {
			uint8_t d __attribute__((unused));
			d = _pkinetisl_spi->DL;
			_data_sent_not_completed--;
		} 
		_pkinetisl_spi->DL = c; // output byte
		_data_sent_not_completed++; // let system know we sent something	

	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) *dataport |=  datapinmask;
			else        *dataport &= ~datapinmask;
			*clkport |=  clkpinmask;
			*clkport &= ~clkpinmask;
		}
	}
}

void ST7735_t3::writecommand(uint8_t c)
{
	setCommandMode();
	spiwrite(c);
}
void ST7735_t3::writecommand_last(uint8_t c)
{
	setCommandMode();
	spiwrite(c);
	waitTransmitComplete();
}

void ST7735_t3::writedata(uint8_t c)
{
	setDataMode();
	spiwrite(c);
} 

void ST7735_t3::writedata_last(uint8_t c)
{
	setDataMode();
	spiwrite(c);
	waitTransmitComplete();
} 

void ST7735_t3::writedata16(uint16_t d)
{
	setDataMode();
	spiwrite16(d);
} 

void ST7735_t3::writedata16_last(uint16_t d)
{
	setDataMode();
	spiwrite16(d);
	waitTransmitComplete();
	_pkinetisl_spi->C2 = 0; // Set back to 8 bit mode...
	_pkinetisl_spi->S;	// Read in the status;
} 

void ST7735_t3::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}
#endif //#if defined(__SAM3X8E__)


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd2green144[] = {         // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127

  Rcmd2green160x80[] = {            // 7735R init, part 2 (mini 160x80)
    2,                              //  2 commands in list:
    ST7735_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x4F,                   //     XEND = 79
    ST7735_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x9F},                 //     XEND = 159
  Rcmd2minist7735s[] = {
    3,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00+26,             //     XSTART = 0
      0x00, 0x7F+26,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00+1,             //     XSTART = 0
      0x00, 0x4F+1,           //     XEND = 79
    ST7735_INVON,  0},			// these displays need colors inversed

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ST7735_t3::commandList(const uint8_t *addr)
{
	uint8_t  numCommands, numArgs;
	uint16_t ms;

	beginSPITransaction();
	numCommands = pgm_read_byte(addr++);		// Number of commands to follow
	//Serial.printf("CommandList: numCmds:%d\n", numCommands); Serial.flush();
	while(numCommands--) {				// For each command...
		writecommand_last(pgm_read_byte(addr++));	//   Read, issue command
		numArgs  = pgm_read_byte(addr++);	//   Number of args to follow
		ms       = numArgs & DELAY;		//   If hibit set, delay follows args
		numArgs &= ~DELAY;			//   Mask out delay bit
		while(numArgs > 1) {			//   For each argument...
			writedata(pgm_read_byte(addr++)); //   Read, issue argument
			numArgs--;
		}

		if (numArgs) writedata_last(pgm_read_byte(addr++)); //   Read, issue argument - wait until this one completes
		if(ms) {
			ms = pgm_read_byte(addr++);	// Read post-command delay time (ms)
			if(ms == 255) ms = 500;		// If 255, delay for 500 ms
			//Serial.printf("delay %d\n", ms); Serial.flush();
			endSPITransaction();
			delay(ms);
			beginSPITransaction();
		}
	}
	endSPITransaction();
}


// Initialization code common to both 'B' and 'R' type displays
void ST7735_t3::commonInit(const uint8_t *cmdList, uint8_t mode)
{
	_colstart  = _rowstart = 0; // May be overridden in init func
  	_ystart = _xstart = 0;

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;

	// Lets try to handle cases where DC is not hardware, without going all the way down to bit bang!
	//if (SPI.pinIsMOSI(_sid) && SPI.pinIsSCK(_sclk) && SPI.pinIsChipSelect(_rs)) {
	if (SPI.pinIsMOSI(_sid) && SPI.pinIsSCK(_sclk)) {
		_pspi = &SPI;
		_spi_num = 0;          // Which buss is this spi on? 
		_pkinetisk_spi = &KINETISK_SPI0;  // Could hack our way to grab this from SPI object, but...
		_fifo_full_test = (3 << 12);
		//Serial.println("ST7735_t3::commonInit SPI");

    #if  defined(__MK64FX512__) || defined(__MK66FX1M0__)
	} else if (SPI1.pinIsMOSI(_sid) && SPI1.pinIsSCK(_sclk)) {
		_pspi = &SPI1;
		_spi_num = 1;          // Which buss is this spi on? 

		_pkinetisk_spi = &KINETISK_SPI1;
		_fifo_full_test = (0 << 12);
		//Serial.println("ST7735_t3::commonInit SPI1");
	} else if (SPI2.pinIsMOSI(_sid) && SPI2.pinIsSCK(_sclk)) {
		_pspi = &SPI2;
		_spi_num = 2;          // Which buss is this spi on? 
		_pkinetisk_spi = &KINETISK_SPI2;
		_fifo_full_test = (0 << 12);
		//Serial.println("ST7735_t3::commonInit SPI2");
    #endif		
	} else _pspi = nullptr;

	if (_pspi) {
		hwSPI = true;
		_pspi->setMOSI(_sid);
		_pspi->setSCK(_sclk);
		_pspi->begin();
		//Serial.println("After SPI begin");
		_spiSettings = SPISettings(ST7735_SPICLOCK, MSBFIRST, mode);
		// See if both CS and DC are valid CS pins.
		if (_pspi->pinIsChipSelect(_rs, _cs)) {
			pcs_data = _pspi->setCS(_cs);
			pcs_command = pcs_data | _pspi->setCS(_rs);
			cspin = 0; // Let know that we are not setting manual
			//Serial.println("Both CS and DC are SPI pins");
		// See if at least DC is hardware CS... 	
		} else if (_pspi->pinIsChipSelect(_rs)) {
			// We already verified that _rs was valid CS pin above.
			pcs_data = 0;
			pcs_command = pcs_data | _pspi->setCS(_rs);
			if (_cs != 0xff) {
				pinMode(_cs, OUTPUT);
				cspin = portOutputRegister(digitalPinToPort(_cs));
				*cspin = 1;
			}
		} else {
			// DC is not, and won't bother to check CS... 
			pcs_data = 0;
			pcs_command = 0;
			if (_cs != 0xff) {
				pinMode(_cs, OUTPUT);
				cspin = portOutputRegister(digitalPinToPort(_cs));
				*cspin = 1;
			}
			pinMode(_rs, OUTPUT);
			rspin = portOutputRegister(digitalPinToPort(_rs));
			*rspin = 0;
			// Pass 1, now lets set hwSPI false
			hwSPI = false;
		}
		// Hack to get hold of the SPI Hardware information... 
	 	uint32_t *pa = (uint32_t*)((void*)_pspi);
		_spi_hardware = (SPIClass::SPI_Hardware_t*)(void*)pa[1];
	} else {
		//Serial.println("ST7735_t3::commonInit Software SPI :(");
		hwSPI = false;
		_pspi = nullptr;
		cspin = (_cs != 0xff)? portOutputRegister(digitalPinToPort(_cs)) : 0;
		rspin = portOutputRegister(digitalPinToPort(_rs));
		clkpin = portOutputRegister(digitalPinToPort(_sclk));
		datapin = portOutputRegister(digitalPinToPort(_sid));
		*cspin = 1;
		*rspin = 0;
		*clkpin = 0;
		*datapin = 0;
		if (_cs != 0xff) pinMode(_cs, OUTPUT);
		pinMode(_rs, OUTPUT);
		pinMode(_sclk, OUTPUT);
		pinMode(_sid, OUTPUT);
	}
	// Teensy 4
#elif defined(__IMXRT1062__)  // Teensy 4.x 
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	if (SPI.pinIsMOSI(_sid) && SPI.pinIsSCK(_sclk)) {
		_pspi = &SPI;
		_spi_num = 0;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI4_S;  // Could hack our way to grab this from SPI object, but...

	} else if (SPI1.pinIsMOSI(_sid) && SPI1.pinIsSCK(_sclk)) {
		_pspi = &SPI1;
		_spi_num = 1;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI3_S;
	} else if (SPI2.pinIsMOSI(_sid) && SPI2.pinIsSCK(_sclk)) {
		_pspi = &SPI2;
		_spi_num = 2;          // Which buss is this spi on? 
		_pimxrt_spi = &IMXRT_LPSPI1_S;
	} else _pspi = nullptr;

	if (_pspi) {
		hwSPI = true;
		_pspi->begin();
		_pending_rx_count = 0;
		_spiSettings = SPISettings(ST7735_SPICLOCK, MSBFIRST, mode);
		_pspi->beginTransaction(_spiSettings); // Should have our settings. 
		_pspi->transfer(0);	// hack to see if it will actually change then...
		_pspi->endTransaction();
		_spi_tcr_current = _pimxrt_spi->TCR; // get the current TCR value 
//		uint32_t *phack = (uint32_t* )&_spiSettings;
//		Serial.printf("SPI Settings: TCR: %x %x (%x %x)\n", _spi_tcr_current, _pimxrt_spi->TCR, phack[0], phack[1]);
		// Hack to get hold of the SPI Hardware information... 
	 	uint32_t *pa = (uint32_t*)((void*)_pspi);
		_spi_hardware = (SPIClass::SPI_Hardware_t*)(void*)pa[1];
	
	} else {
		hwSPI = false;
		_sckport = portOutputRegister(_sclk);
		_sckpinmask = digitalPinToBitMask(_sclk);
		pinMode(_sclk, OUTPUT);	
		DIRECT_WRITE_LOW(_sckport, _sckpinmask);

		_mosiport = portOutputRegister(_sid);
		_mosipinmask = digitalPinToBitMask(_sid);
		pinMode(_sid, OUTPUT);	
		DIRECT_WRITE_LOW(_mosiport, _mosipinmask);

	}
	if (_cs != 0xff) {
		_csport = portOutputRegister(_cs);
		_cspinmask = digitalPinToBitMask(_cs);
		pinMode(_cs, OUTPUT);	
		DIRECT_WRITE_HIGH(_csport, _cspinmask);		
	} else _csport = 0;

	if (_pspi && _pspi->pinIsChipSelect(_rs)) {
	 	uint8_t dc_cs_index = _pspi->setCS(_rs);
	 	_dcport = 0;
	 	_dcpinmask = 0;
	 	dc_cs_index--;	// convert to 0 based
		_tcr_dc_assert = LPSPI_TCR_PCS(dc_cs_index);
    	_tcr_dc_not_assert = LPSPI_TCR_PCS(3);
	} else {
		//Serial.println("ST7735_t3: Error not DC is not valid hardware CS pin");
		_dcport = portOutputRegister(_rs);
		_dcpinmask = digitalPinToBitMask(_rs);
		pinMode(_rs, OUTPUT);	
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		_tcr_dc_assert = LPSPI_TCR_PCS(0);
    	_tcr_dc_not_assert = LPSPI_TCR_PCS(1);
	}
	maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));

    // Teensy LC
#elif defined(__MKL26Z64__)
	// BUGBUG:: cleanup use SPI tables to confirm, like above. 
	hwSPI1 = false;
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	
	// See if pins are on standard SPI0
	if ((_sid == 7 || _sid == 11) && (_sclk == 13 || _sclk == 14)) {
		hwSPI = true;
	} else {
		hwSPI = false;
		if ((_sid == 0 || _sid == 21) && (_sclk == 20 )) {
			hwSPI1 = true;
		}
	}
 
	if (_cs != 0xff) {
		pinMode(_cs, OUTPUT);
		csport    = portOutputRegister(digitalPinToPort(_cs));
		cspinmask = digitalPinToBitMask(_cs);		
	} else csport = 0;

	pinMode(_rs, OUTPUT);
	rsport    = portOutputRegister(digitalPinToPort(_rs));
	rspinmask = digitalPinToBitMask(_rs);
	_spiSettings = SPISettings(ST7735_SPICLOCK, MSBFIRST, mode);

	if(hwSPI) { // Using hardware SPI
		if (_sclk == 14) SPI.setSCK(14);
		if (_sid == 7) SPI.setMOSI(7);
		SPI.begin();
		_pkinetisl_spi = &KINETISL_SPI0;
	} else if(hwSPI1) { // Using hardware SPI
		SPI1.setSCK(_sclk);
		SPI1.setMOSI(_sid);
		SPI1.begin();
		_pkinetisl_spi = &KINETISL_SPI1;
	} else {
		_pkinetisl_spi = nullptr;
		pinMode(_sclk, OUTPUT);
		pinMode(_sid , OUTPUT);
		clkport     = portOutputRegister(digitalPinToPort(_sclk));
		dataport    = portOutputRegister(digitalPinToPort(_sid));
		clkpinmask  = digitalPinToBitMask(_sclk);
		datapinmask = digitalPinToBitMask(_sid);
		*clkport   &= ~clkpinmask;
		*dataport  &= ~datapinmask;
	}
	// toggle RST low to reset; CS low so it'll listen to us
	*csport &= ~cspinmask;

	_dcpinAsserted = 2;	// Should be forced on next call
	_data_sent_not_completed = 0;

#endif
	// BUGBUG
//	digitalWrite(_cs, LOW);
	if (_rst != 0xff) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(100);
		digitalWrite(_rst, LOW);
		delay(100);
		digitalWrite(_rst, HIGH);
		delay(200);
	}

	if(cmdList) commandList(cmdList);
}


// Initialization for ST7735B screens
void ST7735_t3::initB(void)
{
	commonInit(Bcmd);
}


// Initialization for ST7735R screens (green or red tabs)
void ST7735_t3::initR(uint8_t options)
{
	commonInit(Rcmd1);
	if (options == INITR_GREENTAB) {
		commandList(Rcmd2green);
		_colstart = 2;
		_rowstart = 1;
	} else if(options == INITR_144GREENTAB) {
		_screenHeight = ST7735_TFTHEIGHT_144;
		commandList(Rcmd2green144);
		_colstart = 2;
		_rowstart = 3;
	} else if(options == INITR_144GREENTAB_OFFSET) {
		_screenHeight = ST7735_TFTHEIGHT_144;
		commandList(Rcmd2green144);
		_colstart = 0;
		_rowstart = 32;
	} else if(options == INITR_MINI160x80) {
	    _screenHeight   = ST7735_TFTHEIGHT_160;
	    _screenWidth    = ST7735_TFTWIDTH_80;
	    commandList(Rcmd2green160x80);
	    _colstart = 24;
	    _rowstart = 0;
	} else if (options == INITR_MINI160x80_ST7735S) {
	    _screenHeight   = 160;
	    _screenWidth    = 80;
	    commandList(Rcmd2minist7735s);
	    _colstart = 26;
	    _rowstart = 1;
	} else {
		// _colstart, _rowstart left at default '0' values
		commandList(Rcmd2red);
	}
	commandList(Rcmd3);

	// if black or mini, change MADCTL color filter
	if ((options == INITR_BLACKTAB)  || (options == INITR_MINI160x80)){
		writecommand(ST7735_MADCTL);
		writedata_last(0xC0);
	}

	tabcolor = options;
	setRotation(0);	

}


void ST7735_t3::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	beginSPITransaction();
	setAddr(x0, y0, x1, y1);
	writecommand(ST7735_RAMWR); // write to RAM
	// The setAddrWindow/pushColor will only work if SPI is kept active during this loop...
	endSPITransaction();
}


void ST7735_t3::pushColor(uint16_t color, boolean last_pixel)
{
	//beginSPITransaction();
	if (last_pixel) {
		writedata16_last(color);
		endSPITransaction();
	} else {
		writedata16(color);
	}
}

//#include "glcdfont.c"
extern "C" const unsigned char glcdfont[];
 

void ST7735_t3::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	x += _originx;
	y += _originy;
	
	if((x < _displayclipx1) ||(x >= _displayclipx2) || (y < _displayclipy1) || (y >= _displayclipy2)) return;

	#ifdef ENABLE_ST77XX_FRAMEBUFFER
	if (_use_fbtft) {
		_pfbtft[y*_width + x] = color;

	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x,y,x+1,y+1);
		writecommand(ST7735_RAMWR);
		writedata16_last(color);
		endSPITransaction();
	}
}


void ST7735_t3::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	x+=_originx;
	y+=_originy;
	// Rectangular clipping
	if((x < _displayclipx1) || (x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if(y < _displayclipy1) { h = h - (_displayclipy1 - y); y = _displayclipy1;}
	if((y+h-1) >= _displayclipy2) h = _displayclipy2-y;
	if(h<1) return;

	#ifdef ENABLE_ST77XX_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel = &_pfbtft[ y*_width + x];
		while (h--) {
			*pfbPixel = color;
			pfbPixel += _width;
		}
	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x, y+h-1);
		writecommand(ST7735_RAMWR);
		while (h-- > 1) {
			writedata16(color);
		}
		writedata16_last(color);
		endSPITransaction();
	}
}


void ST7735_t3::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	x+=_originx;
	y+=_originy;

	// Rectangular clipping
	if((y < _displayclipy1) || (x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if(x<_displayclipx1) { w = w - (_displayclipx1 - x); x = _displayclipx1; }
	if((x+w-1) >= _displayclipx2)  w = _displayclipx2-x;
	if (w<1) return;

	#ifdef ENABLE_ST77XX_FRAMEBUFFER
	if (_use_fbtft) {
		if ((x&1) || (w&1)) {
			uint16_t * pfbPixel = &_pfbtft[ y*_width + x];
			while (w--) {
				*pfbPixel++ = color;
			}
		} else {
			// X is even and so is w, try 32 bit writes..
			uint32_t color32 = (color << 16) | color;
			uint32_t * pfbPixel = (uint32_t*)((uint16_t*)&_pfbtft[ y*_width + x]);
			while (w) {
				*pfbPixel++ = color32;
				w -= 2;
			}
		}
	} else 
	#endif
	{
		beginSPITransaction();
		setAddr(x, y, x+w-1, y);
		writecommand(ST7735_RAMWR);
		while (w-- > 1) {
			writedata16(color);
		}
		writedata16_last(color);
		endSPITransaction();
	}
}

void ST7735_t3::fillScreen(uint16_t color)
{
	fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void ST7735_t3::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	x+=_originx;
	y+=_originy;

	// Rectangular clipping (drawChar w/big text requires this)
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;
	if(x < _displayclipx1) {	w -= (_displayclipx1-x); x = _displayclipx1; 	}
	if(y < _displayclipy1) {	h -= (_displayclipy1 - y); y = _displayclipy1; 	}
	if((x + w - 1) >= _displayclipx2)  w = _displayclipx2  - x;
	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

	#ifdef ENABLE_ST77XX_FRAMEBUFFER
	if (_use_fbtft) {
		if ((x&1) || (w&1)) {
			uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
			for (;h>0; h--) {
				uint16_t * pfbPixel = pfbPixel_row;
				for (int i = 0 ;i < w; i++) {
					*pfbPixel++ = color;
				}
				pfbPixel_row += _width;
			}
		} else {
			// Horizontal is even number so try 32 bit writes instead
			uint32_t color32 = (color << 16) | color;
			uint32_t * pfbPixel_row = (uint32_t *)((uint16_t*)&_pfbtft[ y*_width + x]);
			w = w/2;	// only iterate half the times
			for (;h>0; h--) {
				uint32_t * pfbPixel = pfbPixel_row;
				for (int i = 0 ;i < w; i++) {
					*pfbPixel++ = color32;
				}
				pfbPixel_row += (_width/2);
			}
		}
	} else 
	#endif
	{

		// TODO: this can result in a very long transaction time
		// should break this into multiple transactions, even though
		// it'll cost more overhead, so we don't stall other SPI libs
		beginSPITransaction();
		setAddr(x, y, x+w-1, y+h-1);
		writecommand(ST7735_RAMWR);
		for(y=h; y>0; y--) {
			for(x=w; x>1; x--) {
				writedata16(color);
			}
			writedata16_last(color);		
		}
		endSPITransaction();
	}
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ST7735_t3::setRotation(uint8_t m)
{
	//Serial.printf("Setting Rotation to %d\n", m);
	beginSPITransaction();
	writecommand(ST7735_MADCTL);
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
     	if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
			writedata_last(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
		} else {
			writedata_last(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
		}
		_width  = _screenWidth;
		_height = _screenHeight;
	    _xstart = _colstart;
	    _ystart = _rowstart;
		break;
	case 1:
     	if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
			writedata_last(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
		} else {
			writedata_last(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		}
		_height = _screenWidth;
		_width  = _screenHeight;
     	_ystart = _colstart;
     	_xstart = _rowstart;
		break;
	case 2:
     	if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
			writedata_last(MADCTL_RGB);
		} else {
			writedata_last(MADCTL_BGR);
		}
		_width  = _screenWidth;
		_height = _screenHeight;
     	_xstart = _colstart;
     	// hack to make work on a couple different displays
     	_ystart = (_rowstart==0 || _rowstart==32)? 0 : 1;//_rowstart;
		break;
	case 3:
     	if ((tabcolor == INITR_BLACKTAB) || (tabcolor == INITR_MINI160x80)) {
			writedata_last(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
		} else {
			writedata_last(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		}
		_width = _screenHeight;
		_height = _screenWidth;
     	_ystart = _colstart;
     	// hack to make work on a couple different displays
     	_xstart = (_rowstart==0 || _rowstart==32)? 0 : 1;//_rowstart;
		break;
	}
	_rot = rotation;	// remember the rotation... 
	endSPITransaction();

	//Serial.printf("SetRotation(%d) _xstart=%d _ystart=%d _width=%d, _height=%d\n", _rot, _xstart, _ystart, _width, _height);

	
	setClipRect();
	setOrigin();
	
	cursor_x = 0;
	cursor_y = 0;
}

void ST7735_t3::setRowColStart(uint16_t x, uint16_t y) {
	_rowstart = x;
	_colstart = y;
	if (_rot != 0xff) setRotation(_rot);
}


void ST7735_t3::invertDisplay(boolean i)
{
	beginSPITransaction();
	writecommand_last(i ? ST7735_INVON : ST7735_INVOFF);
	endSPITransaction();

}

/*!
 @brief   Adafruit_SPITFT Send Command handles complete sending of commands and const data
 @param   commandByte       The Command Byte
 @param   dataBytes         A pointer to the Data bytes to send
 @param   numDataBytes      The number of bytes we should send
 */
void ST7735_t3::sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes) {
    beginSPITransaction();

    writecommand_last(commandByte); // Send the command byte
  
    while (numDataBytes > 1) {
	  writedata(*dataBytes++); // Send the data bytes
	  numDataBytes--;
    }
    if (numDataBytes) writedata_last(*dataBytes);
  
    endSPITransaction();
}

uint16_t ST7735_t3::readPixel(int16_t x, int16_t y)
{
	uint16_t colors = 0;
	readRect(x, y, 1, 1, &colors);
	return colors;
}


// Now lets see if we can read in multiple pixels
void ST7735_t3::readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors)
{
	// Use our Origin. 
	x+=_originx;
	y+=_originy;
	//BUGBUG:: Should add some validation of X and Y

	#ifdef ENABLE_ST77XX_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
		for (;h>0; h--) {
			uint16_t * pfbPixel = pfbPixel_row;
			for (int i = 0 ;i < w; i++) {
				*pcolors++ = *pfbPixel++;
			}
			pfbPixel_row += _width;
		}
		return;  
	}
	#endif   
}

// Now lets see if we can writemultiple pixels
void ST7735_t3::writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors)
{
	if (x == CENTER)
		x = (_width - w) / 2;
	if (y == CENTER)
		y = (_height - h) / 2;

	x+=_originx;
	y+=_originy;

	uint16_t x_clip_left = 0;  // How many entries at start of colors to skip at start of row
	uint16_t x_clip_right = 0;    // how many color entries to skip at end of row for clipping
	// Rectangular clipping 

	// See if the whole thing out of bounds...
	if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
	if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

	// In these cases you can not do simple clipping, as we need to synchronize the colors array with the
	// We can clip the height as when we get to the last visible we don't have to go any farther. 
	// also maybe starting y as we will advance the color array. 
 	if(y < _displayclipy1) {
 		int dy = (_displayclipy1 - y);
 		h -= dy; 
 		pcolors += (dy*w); // Advance color array to 
 		y = _displayclipy1; 	
 	}
	if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;
	// For X see how many items in color array to skip at start of row and likewise end of row 
	if(x < _displayclipx1) {
		x_clip_left = _displayclipx1-x; 
		w -= x_clip_left; 
		x = _displayclipx1; 	
	}

	if((x + w - 1) >= _displayclipx2) {
		x_clip_right = w;
		w = _displayclipx2  - x;
		x_clip_right -= w; 
	} 

	#ifdef ENABLE_ST77XX_FRAMEBUFFER
	if (_use_fbtft) {
		uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
		for (;h>0; h--) {
			uint16_t * pfbPixel = pfbPixel_row;
			pcolors += x_clip_left;
			for (int i = 0 ;i < w; i++) {
				*pfbPixel++ = *pcolors++;
			}
			pfbPixel_row += _width;
			pcolors += x_clip_right;

		}
		return;	
	}
	#endif

   	beginSPITransaction();
	setAddr(x, y, x+w-1, y+h-1);
	writecommand(ST7735_RAMWR);
	for(y=h; y>0; y--) {
		pcolors += x_clip_left;
		for(x=w; x>1; x--) {
			writedata16(*pcolors++);
		}
		writedata16_last(*pcolors++);
		pcolors += x_clip_right;
	}
	endSPITransaction();
}

void ST7735_t3::writeSubImageRect(int16_t x, int16_t y, int16_t w, int16_t h, 
  int16_t image_offset_x, int16_t image_offset_y, int16_t image_width, int16_t image_height, const uint16_t *pcolors)
{
  if (x == CENTER) x = (_width - w) / 2;
  if (y == CENTER) y = (_height - h) / 2;
  x+=_originx;
  y+=_originy;
  // Rectangular clipping 

  // See if the whole thing out of bounds...
  if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
  if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

  // Now lets use or image offsets to get to the first pixels data
  pcolors += image_offset_y * image_width + image_offset_x;

  // In these cases you can not do simple clipping, as we need to synchronize the colors array with the
  // We can clip the height as when we get to the last visible we don't have to go any farther. 
  // also maybe starting y as we will advance the color array. 
  if(y < _displayclipy1) {
    int dy = (_displayclipy1 - y);
    h -= dy; 
    pcolors += (dy * image_width); // Advance color array by that number of rows in the image 
    y = _displayclipy1;   
  }

  if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

  // For X see how many items in color array to skip at start of row and likewise end of row 
  if(x < _displayclipx1) {
    uint16_t x_clip_left = _displayclipx1-x; 
    w -= x_clip_left; 
    x = _displayclipx1;   
    pcolors += x_clip_left;  // pre index the colors array.
  }
  if((x + w - 1) >= _displayclipx2) {
    uint16_t x_clip_right = w;
    w = _displayclipx2  - x;
    x_clip_right -= w; 
  } 

  #ifdef ENABLE_ST77XX_FRAMEBUFFER
  if (_use_fbtft) {
    uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
    for (;h>0; h--) {
      const uint16_t *pcolors_row = pcolors; 
      uint16_t * pfbPixel = pfbPixel_row;
      for (int i = 0 ;i < w; i++) {
        *pfbPixel++ = *pcolors++;
      }
      pfbPixel_row += _width;
      pcolors = pcolors_row + image_width;
    }
    return; 
  }
  #endif

  beginSPITransaction();
  setAddr(x, y, x+w-1, y+h-1);
  writecommand(ST7735_RAMWR);
  for(y=h; y>0; y--) {
    const uint16_t *pcolors_row = pcolors; 
    for(x=w; x>1; x--) {
      writedata16(*pcolors++);
    }
    writedata16_last(*pcolors++);
    pcolors = pcolors_row + image_width;
  }
  endSPITransaction();
}

void ST7735_t3::writeSubImageRectBytesReversed(int16_t x, int16_t y, int16_t w, int16_t h, 
  int16_t image_offset_x, int16_t image_offset_y, int16_t image_width, int16_t image_height, const uint16_t *pcolors)
{
  if (x == CENTER) x = (_width - w) / 2;
  if (y == CENTER) y = (_height - h) / 2;
  x+=_originx;
  y+=_originy;
  // Rectangular clipping 

  // See if the whole thing out of bounds...
  if((x >= _displayclipx2) || (y >= _displayclipy2)) return;
  if (((x+w) <= _displayclipx1) || ((y+h) <= _displayclipy1)) return;

  // Now lets use or image offsets to get to the first pixels data
  pcolors += image_offset_y * image_width + image_offset_x;

  // In these cases you can not do simple clipping, as we need to synchronize the colors array with the
  // We can clip the height as when we get to the last visible we don't have to go any farther. 
  // also maybe starting y as we will advance the color array. 
  if(y < _displayclipy1) {
    int dy = (_displayclipy1 - y);
    h -= dy; 
    pcolors += (dy * image_width); // Advance color array by that number of rows in the image 
    y = _displayclipy1;   
  }

  if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

  // For X see how many items in color array to skip at start of row and likewise end of row 
  if(x < _displayclipx1) {
    uint16_t x_clip_left = _displayclipx1-x; 
    w -= x_clip_left; 
    x = _displayclipx1;   
    pcolors += x_clip_left;  // pre index the colors array.
  }
  if((x + w - 1) >= _displayclipx2) {
    uint16_t x_clip_right = w;
    w = _displayclipx2  - x;
    x_clip_right -= w; 
  } 

  #ifdef ENABLE_ST77XX_FRAMEBUFFER
  if (_use_fbtft) {
    uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
    for (;h>0; h--) {
      const uint16_t *pcolors_row = pcolors; 
      uint16_t * pfbPixel = pfbPixel_row;
      for (int i = 0 ;i < w; i++) {
        *pfbPixel++ = *pcolors++;
      }
      pfbPixel_row += _width;
      pcolors = pcolors_row + image_width;
    }
    return; 
  }
  #endif

  beginSPITransaction();
  setAddr(x, y, x+w-1, y+h-1);
  writecommand(ST7735_RAMWR);
  for(y=h; y>0; y--) {
    const uint16_t *pcolors_row = pcolors; 
    for(x=w; x>1; x--) {
      uint16_t color = *pcolors++;
      color = ((color & 0xff) << 8) + (color >> 8);
      writedata16(color);
    }
      uint16_t color = *pcolors;
      color = ((color & 0xff) << 8) + (color >> 8);
      writedata16_last(color);
    pcolors = pcolors_row + image_width;
  }
  endSPITransaction();
}

// writeRect8BPP - 	write 8 bit per pixel paletted bitmap
//					bitmap data in array at pixels, one byte per
//pixel
//					color palette data in array at palette
void ST7735_t3::writeRect8BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint8_t *pixels,
                                const uint16_t *palette) {
  // Serial.printf("\nWR8: %d %d %d %d %x\n", x, y, w, h, (uint32_t)pixels);
  x += _originx;
  y += _originy;

  uint16_t x_clip_left =
      0; // How many entries at start of colors to skip at start of row
  uint16_t x_clip_right =
      0; // how many color entries to skip at end of row for clipping
  // Rectangular clipping

  // See if the whole thing out of bounds...
  if ((x >= _displayclipx2) || (y >= _displayclipy2))
    return;
  if (((x + w) <= _displayclipx1) || ((y + h) <= _displayclipy1))
    return;

  // In these cases you can not do simple clipping, as we need to synchronize
  // the colors array with the
  // We can clip the height as when we get to the last visible we don't have to
  // go any farther.
  // also maybe starting y as we will advance the color array.
  if (y < _displayclipy1) {
    int dy = (_displayclipy1 - y);
    h -= dy;
    pixels += (dy * w); // Advance color array to
    y = _displayclipy1;
  }

  if ((y + h - 1) >= _displayclipy2)
    h = _displayclipy2 - y;

  // For X see how many items in color array to skip at start of row and
  // likewise end of row
  if (x < _displayclipx1) {
    x_clip_left = _displayclipx1 - x;
    w -= x_clip_left;
    x = _displayclipx1;
  }
  if ((x + w - 1) >= _displayclipx2) {
    x_clip_right = w;
    w = _displayclipx2 - x;
    x_clip_right -= w;
  }
// Serial.printf("WR8C: %d %d %d %d %x- %d %d\n", x, y, w, h, (uint32_t)pixels,
// x_clip_right, x_clip_left);
#ifdef ENABLE_ST77XX_FRAMEBUFFER
  if (_use_fbtft) {
    uint16_t *pfbPixel_row = &_pfbtft[y * _width + x];
    for (; h > 0; h--) {
      pixels += x_clip_left;
      uint16_t *pfbPixel = pfbPixel_row;
      for (int i = 0; i < w; i++) {
        *pfbPixel++ = palette[*pixels++];
      }
      pixels += x_clip_right;
      pfbPixel_row += _width;
    }
    return;
  }
#endif

  beginSPITransaction();
  setAddr(x, y, x + w - 1, y + h - 1);
  writecommand(ST7735_RAMWR);
  for (y = h; y > 0; y--) {
    pixels += x_clip_left;
    // Serial.printf("%x: ", (uint32_t)pixels);
    for (x = w; x > 1; x--) {
      // Serial.print(*pixels, DEC);
      writedata16(palette[*pixels++]);
    }
    // Serial.println(*pixels, DEC);
    writedata16_last(palette[*pixels++]);
    pixels += x_clip_right;
  }
  endSPITransaction();
}

// writeRect4BPP - 	write 4 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per
//pixel
//					color palette data in array at palette
//					width must be at least 2 pixels
void ST7735_t3::writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint8_t *pixels,
                                const uint16_t *palette) {
  // Simply call through our helper
  writeRectNBPP(x, y, w, h, 4, pixels, palette);
}

// writeRect2BPP - 	write 2 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per
//pixel
//					color palette data in array at palette
//					width must be at least 4 pixels
void ST7735_t3::writeRect2BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint8_t *pixels,
                                const uint16_t *palette) {
  // Simply call through our helper
  writeRectNBPP(x, y, w, h, 2, pixels, palette);
}

///============================================================================
// writeRect1BPP - 	write 1 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per
//pixel
//					color palette data in array at palette
//					width must be at least 8 pixels
void ST7735_t3::writeRect1BPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint8_t *pixels,
                                const uint16_t *palette) {
  // Simply call through our helper
  writeRectNBPP(x, y, w, h, 1, pixels, palette);
}

///============================================================================
// writeRectNBPP - 	write N(1, 2, 4, 8) bit per pixel paletted bitmap
//					bitmap data in array at pixels
//  Currently writeRect1BPP, writeRect2BPP, writeRect4BPP use this to do all of
//  the work.
void ST7735_t3::writeRectNBPP(int16_t x, int16_t y, int16_t w, int16_t h,
                                uint8_t bits_per_pixel, const uint8_t *pixels,
                                const uint16_t *palette) {
  // Serial.printf("\nWR8: %d %d %d %d %x\n", x, y, w, h, (uint32_t)pixels);
  x += _originx;
  y += _originy;
  uint8_t pixels_per_byte = 8 / bits_per_pixel;
  uint16_t count_of_bytes_per_row =
      (w + pixels_per_byte - 1) /
      pixels_per_byte; // Round up to handle non multiples
  uint8_t row_shift_init =
      8 - bits_per_pixel; // We shift down 6 bits by default
  uint8_t pixel_bit_mask = (1 << bits_per_pixel) - 1; // get mask to use below
  // Rectangular clipping

  // See if the whole thing out of bounds...
  if ((x >= _displayclipx2) || (y >= _displayclipy2))
    return;
  if (((x + w) <= _displayclipx1) || ((y + h) <= _displayclipy1))
    return;

  // In these cases you can not do simple clipping, as we need to synchronize
  // the colors array with the
  // We can clip the height as when we get to the last visible we don't have to
  // go any farther.
  // also maybe starting y as we will advance the color array.
  // Again assume multiple of 8 for width
  if (y < _displayclipy1) {
    int dy = (_displayclipy1 - y);
    h -= dy;
    pixels += dy * count_of_bytes_per_row;
    y = _displayclipy1;
  }

  if ((y + h - 1) >= _displayclipy2)
    h = _displayclipy2 - y;

  // For X see how many items in color array to skip at start of row and
  // likewise end of row
  if (x < _displayclipx1) {
    uint16_t x_clip_left = _displayclipx1 - x;
    w -= x_clip_left;
    x = _displayclipx1;
    // Now lets update pixels to the rigth offset and mask
    uint8_t x_clip_left_bytes_incr = x_clip_left / pixels_per_byte;
    pixels += x_clip_left_bytes_incr;
    row_shift_init =
        8 -
        (x_clip_left - (x_clip_left_bytes_incr * pixels_per_byte) + 1) *
            bits_per_pixel;
  }

  if ((x + w - 1) >= _displayclipx2) {
    w = _displayclipx2 - x;
  }

  const uint8_t *pixels_row_start =
      pixels; // remember our starting position offset into row

#ifdef ENABLE_ST77XX_FRAMEBUFFER
  if (_use_fbtft) {
    uint16_t *pfbPixel_row = &_pfbtft[y * _width + x];
    for (; h > 0; h--) {
      uint16_t *pfbPixel = pfbPixel_row;
      pixels = pixels_row_start;            // setup for this row
      uint8_t pixel_shift = row_shift_init; // Setup mask

      for (int i = 0; i < w; i++) {
        *pfbPixel++ = palette[((*pixels) >> pixel_shift) & pixel_bit_mask];
        if (!pixel_shift) {
          pixel_shift = 8 - bits_per_pixel; // setup next mask
          pixels++;
        } else {
          pixel_shift -= bits_per_pixel;
        }
      }
      pfbPixel_row += _width;
      pixels_row_start += count_of_bytes_per_row;
    }
    return;
  }
#endif

  beginSPITransaction();
  setAddr(x, y, x + w - 1, y + h - 1);
  writecommand(ST7735_RAMWR);
  for (; h > 0; h--) {
    pixels = pixels_row_start;            // setup for this row
    uint8_t pixel_shift = row_shift_init; // Setup mask

    for (int i = 0; i < w; i++) {
      writedata16(palette[((*pixels) >> pixel_shift) & pixel_bit_mask]);
      if (!pixel_shift) {
        pixel_shift = 8 - bits_per_pixel; // setup next mask
        pixels++;
      } else {
        pixel_shift -= bits_per_pixel;
      }
    }
    pixels_row_start += count_of_bytes_per_row;
  }
  writecommand_last(ST7735_NOP);
  endSPITransaction();
}


///
///
///
// Draw a rectangle
void ST7735_t3::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{

		drawFastHLine(x, y, w, color);
		drawFastHLine(x, y+h-1, w, color);
		drawFastVLine(x, y, h, color);
		drawFastVLine(x+w-1, y, h, color);
}

// Draw a rounded rectangle
void ST7735_t3::drawRoundRect(int16_t x, int16_t y, int16_t w,
  int16_t h, int16_t r, uint16_t color) {
  // smarter version
  drawFastHLine(x+r  , y    , w-2*r, color); // Top
  drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  drawFastVLine(x    , y+r  , h-2*r, color); // Left
  drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// Fill a rounded rectangle
void ST7735_t3::fillRoundRect(int16_t x, int16_t y, int16_t w,
				 int16_t h, int16_t r, uint16_t color) {
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

// Draw a triangle
void ST7735_t3::drawTriangle(int16_t x0, int16_t y0,
				int16_t x1, int16_t y1,
				int16_t x2, int16_t y2, uint16_t color) {
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
void ST7735_t3::fillTriangle ( int16_t x0, int16_t y0,
				  int16_t x1, int16_t y1,
				  int16_t x2, int16_t y2, uint16_t color) {

  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    st7735_swap(y0, y1); st7735_swap(x0, x1);
  }
  if (y1 > y2) {
    st7735_swap(y2, y1); st7735_swap(x2, x1);
  }
  if (y0 > y1) {
    st7735_swap(y0, y1); st7735_swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  int32_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) st7735_swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) st7735_swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }
}

// Draw a circle outline
void ST7735_t3::drawCircle(int16_t x0, int16_t y0, int16_t r,
    uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0  , y0+r, color);
  drawPixel(x0  , y0-r, color);
  drawPixel(x0+r, y0  , color);
  drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void ST7735_t3::drawCircleHelper( int16_t x0, int16_t y0,
               int16_t r, uint8_t cornername, uint16_t color) {
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4) {
      drawPixel(x0 + x, y0 + y, color);
      drawPixel(x0 + y, y0 + x, color);
    }
    if (cornername & 0x2) {
      drawPixel(x0 + x, y0 - y, color);
      drawPixel(x0 + y, y0 - x, color);
    }
    if (cornername & 0x8) {
      drawPixel(x0 - y, y0 + x, color);
      drawPixel(x0 - x, y0 + y, color);
    }
    if (cornername & 0x1) {
      drawPixel(x0 - y, y0 - x, color);
      drawPixel(x0 - x, y0 - y, color);
    }
  }
}

void ST7735_t3::fillCircle(int16_t x0, int16_t y0, int16_t r,
			      uint16_t color) {
  drawFastVLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void ST7735_t3::fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
    uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}

// Bresenham's algorithm - thx wikpedia
void ST7735_t3::drawLine(int16_t x0, int16_t y0,
	int16_t x1, int16_t y1, uint16_t color)
{
	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		} else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		} else {
			drawPixel(x0, y0, color);
		}
		return;
	} else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		} else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		st7735_swap(x0, y0);
		st7735_swap(x1, y1);
	}
	if (x0 > x1) {
		st7735_swap(x0, x1);
		st7735_swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	#ifdef ENABLE_ST77XX_FRAMEBUFFER
  	if (!_use_fbtft) beginSPITransaction();
  	#else
	beginSPITransaction();
  	#endif
	int16_t xbegin = x0;
	if (steep) {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					VLine(y0, xbegin, len + 1, color);
				} else {
					Pixel(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			VLine(y0, xbegin, x0 - xbegin, color);
		}

	} else {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					HLine(xbegin, y0, len + 1, color);
				} else {
					Pixel(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			HLine(xbegin, y0, x0 - xbegin, color);
		}
	}
	#ifdef ENABLE_ST77XX_FRAMEBUFFER
  	if (!_use_fbtft) {
		writecommand_last(ST7735_NOP);
		endSPITransaction();
  	}
  	#else
		writecommand_last(ST7735_NOP);
	endSPITransaction();
  	#endif
}

void ST7735_t3::drawBitmap(int16_t x, int16_t y,
			      const uint8_t *bitmap, int16_t w, int16_t h,
			      uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
	drawPixel(x+i, y+j, color);
      }
    }
  }
}

void ST7735_t3::setCursor(int16_t x, int16_t y, bool autoCenter) {
	_center_x_text = autoCenter;	// remember the state. 
	_center_y_text = autoCenter;	// remember the state. 
	if (x == ST7735_t3::CENTER) {
		_center_x_text = true;
		x = _width/2;
	}
	if (y == ST7735_t3::CENTER) {
		_center_y_text = true;
		y = _height/2;
	}
	if (x < 0) x = 0;
	else if (x >= _width) x = _width - 1;
	cursor_x = x;
	if (y < 0) y = 0;
	else if (y >= _height) y = _height - 1;
	cursor_y = y;
	
	if(x>=scroll_x && x<=(scroll_x+scroll_width) && y>=scroll_y && y<=(scroll_y+scroll_height)){
		isWritingScrollArea	= true;
	} else {
		isWritingScrollArea = false;
	}
	_gfx_last_char_x_write = 0;	// Don't use cached data here

}

void ST7735_t3::getCursor(int16_t *x, int16_t *y) {
  *x = cursor_x;
  *y = cursor_y;
}

void ST7735_t3::setTextSize(uint8_t s_x, uint8_t s_y) {
    textsize_x = (s_x > 0) ? s_x : 1;
    textsize_y = (s_y > 0) ? s_y : 1;
}

uint8_t ST7735_t3::getTextSize() {
	return textsize_x;	// bug bug 2 values now
}

uint8_t ST7735_t3::getTextSizeX() {
	return textsize_x;
}
uint8_t ST7735_t3::getTextSizeY() {
	return textsize_y;
}

void ST7735_t3::setTextColor(uint16_t c) {
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

void ST7735_t3::setTextColor(uint16_t c, uint16_t b) {
  textcolor   = c;
  textbgcolor = b;
  // pre-expand colors for fast alpha-blending later
  textcolorPrexpanded = (textcolor | (textcolor << 16)) & 0b00000111111000001111100000011111;
  textbgcolorPrexpanded = (textbgcolor | (textbgcolor << 16)) & 0b00000111111000001111100000011111;
}

void ST7735_t3::setTextWrap(boolean w) {
  wrap = w;
}

boolean ST7735_t3::getTextWrap()
{
	return wrap;
}

uint8_t ST7735_t3::getRotation(void) {
  return _rot;
}


/***************************************************************************************
** Function name:           setTextDatum
** Description:             Set the text position reference datum
***************************************************************************************/
void ST7735_t3::setTextDatum(uint8_t d)
{
  textdatum = d;
}


/***************************************************************************************
** Function name:           drawNumber
** Description:             draw a long integer
***************************************************************************************/
int16_t ST7735_t3::drawNumber(long long_num, int poX, int poY)
{
  char str[14];
  ltoa(long_num, str, 10);
  return drawString(str, poX, poY);
}


int16_t ST7735_t3::drawFloat(float floatNumber, int dp, int poX, int poY)
{
  char str[14];               // Array to contain decimal string
  uint8_t ptr = 0;            // Initialise pointer for array
  int8_t  digits = 1;         // Count the digits to avoid array overflow
  float rounding = 0.5;       // Round up down delta

  if (dp > 7) dp = 7; // Limit the size of decimal portion

  // Adjust the rounding value
  for (uint8_t i = 0; i < dp; ++i) rounding /= 10.0f;

  if (floatNumber < -rounding)    // add sign, avoid adding - sign to 0.0!
  {
    str[ptr++] = '-'; // Negative number
    str[ptr] = 0; // Put a null in the array as a precaution
    digits = 0;   // Set digits to 0 to compensate so pointer value can be used later
    floatNumber = -floatNumber; // Make positive
  }

  floatNumber += rounding; // Round up or down

  // For error put ... in string and return (all TFT_ILI9341_ESP library fonts contain . character)
  if (floatNumber >= 2147483647) {
    strcpy(str, "...");
    //return drawString(str, poX, poY);
  }
  // No chance of overflow from here on

  // Get integer part
  unsigned long temp = (unsigned long)floatNumber;

  // Put integer part into array
  ltoa(temp, str + ptr, 10);

  // Find out where the null is to get the digit count loaded
  while ((uint8_t)str[ptr] != 0) ptr++; // Move the pointer along
  digits += ptr;                  // Count the digits

  str[ptr++] = '.'; // Add decimal point
  str[ptr] = '0';   // Add a dummy zero
  str[ptr + 1] = 0; // Add a null but don't increment pointer so it can be overwritten

  // Get the decimal portion
  floatNumber = floatNumber - temp;

  // Get decimal digits one by one and put in array
  // Limit digit count so we don't get a false sense of resolution
  uint8_t i = 0;
  while ((i < dp) && (digits < 9)) // while (i < dp) for no limit but array size must be increased
  {
    i++;
    floatNumber *= 10;       // for the next decimal
    temp = floatNumber;      // get the decimal
    ltoa(temp, str + ptr, 10);
    ptr++; digits++;         // Increment pointer and digits count
    floatNumber -= temp;     // Remove that digit
  }

  // Finally we can plot the string and return pixel length
  return drawString(str, poX, poY);
}

/***************************************************************************************
** Function name:           drawString (with or without user defined font)
** Description :            draw string with padding if it is defined
***************************************************************************************/
// Without font number, uses font set by setTextFont()
int16_t ST7735_t3::drawString(const String& string, int poX, int poY)
{
  int16_t len = string.length() + 2;
  char buffer[len];
  string.toCharArray(buffer, len);
  return drawString1(buffer, len, poX, poY);
}

int16_t ST7735_t3::drawString1(char string[], int16_t len, int poX, int poY)
{
  int16_t sumX = 0;
  uint8_t padding = 1/*, baseline = 0*/;
  
  uint16_t cwidth = strPixelLen(string); // Find the pixel width of the string in the font
  uint16_t cheight = textsize_y*8;

  
  if (textdatum || padX)
  {
    switch(textdatum) {
      case TC_DATUM:
        poX -= cwidth/2;
        padding += 1;
        break;
      case TR_DATUM:
        poX -= cwidth;
        padding += 2;
        break;
      case ML_DATUM:
        poY -= cheight/2;
        //padding += 0;
        break;
      case MC_DATUM:
        poX -= cwidth/2;
        poY -= cheight/2;
        padding += 1;
        break;
      case MR_DATUM:
        poX -= cwidth;
        poY -= cheight/2;
        padding += 2;
        break;
      case BL_DATUM:
        poY -= cheight;
        //padding += 0;
        break;
      case BC_DATUM:
        poX -= cwidth/2;
        poY -= cheight;
        padding += 1;
        break;
      case BR_DATUM:
        poX -= cwidth;
        poY -= cheight;
        padding += 2;
        break;
	 /*
      case L_BASELINE:
        poY -= baseline;
        //padding += 0;
        break;
      case C_BASELINE:
        poX -= cwidth/2;
        poY -= baseline;
        //padding += 1;
        break;
      case R_BASELINE:
        poX -= cwidth;
        poY -= baseline;
        padding += 2;
        break;
	*/
    }
    // Check coordinates are OK, adjust if not
    if (poX < 0) poX = 0;
    if (poX+cwidth > width())   poX = width() - cwidth;
    if (poY < 0) poY = 0;
    //if (poY+cheight-baseline >_height) poY = _height - cheight;
  }
  if(font == NULL){
	  for(uint8_t i = 0; i < len-2; i++){
		drawChar((int16_t) (poX+sumX), (int16_t) poY, string[i], textcolor, textbgcolor, textsize_x, textsize_y);
		sumX += cwidth/(len-2) + padding;
	  }
  } else {
	  setCursor(poX, poY);
	  for(uint8_t i = 0; i < len-2; i++){
		drawFontChar(string[i]);
		setCursor(cursor_x, cursor_y);
	  }
  }
return sumX;
}


void ST7735_t3::scrollTextArea(uint8_t scrollSize){
	uint16_t awColors[scroll_width];
	for (int y=scroll_y+scrollSize; y < (scroll_y+scroll_height); y++) { 
		readRect(scroll_x, y, scroll_width, 1, awColors); 
		writeRect(scroll_x, y-scrollSize, scroll_width, 1, awColors);  
	}
	fillRect(scroll_x, (scroll_y+scroll_height)-scrollSize, scroll_width, scrollSize, scrollbgcolor);
}

void ST7735_t3::setScrollTextArea(int16_t x, int16_t y, int16_t w, int16_t h){
	scroll_x = x; 
	scroll_y = y;
	scroll_width = w; 
	scroll_height = h;
}

void ST7735_t3::setScrollBackgroundColor(uint16_t color){
	scrollbgcolor=color;
	fillRect(scroll_x,scroll_y,scroll_width,scroll_height,scrollbgcolor);
}

void ST7735_t3::enableScroll(void){
	scrollEnable = true;
}

void ST7735_t3::disableScroll(void){
	scrollEnable = false;
}

void ST7735_t3::resetScrollBackgroundColor(uint16_t color){
	scrollbgcolor=color;
}	


// overwrite functions from class Print:

size_t ST7735_t3::write(uint8_t c) {
	return write(&c, 1);
}

size_t ST7735_t3::write(const uint8_t *buffer, size_t size)
{
	// Lets try to handle some of the special font centering code that was done for default fonts.
	if (_center_x_text || _center_y_text ) {
		int16_t x, y;
	  	uint16_t strngWidth, strngHeight;
	  	getTextBounds(buffer, size, 0, 0, &x, &y, &strngWidth, &strngHeight);
	  	//Serial.printf("_fontwrite bounds: %d %d %u %u\n", x, y, strngWidth, strngHeight);
	  	// Note we may want to play with the x ane y returned if they offset some
		if (_center_x_text && strngWidth > 0){//Avoid operations for strngWidth = 0
      		cursor_x -= (x + strngWidth / 2);
		}
		if (_center_y_text && strngHeight > 0){//Avoid operations for strngWidth = 0
		    cursor_y -= (y + strngHeight / 2);
		}
		_center_x_text = false;
		_center_y_text = false;
	}

	size_t cb = size;
	while (cb) {
		uint8_t c = *buffer++;
		cb--;

		if (font) {
			if (c == '\n') {
				cursor_y += font->line_space;
				if(scrollEnable && isWritingScrollArea){
					cursor_x  = scroll_x;
				}else{
					cursor_x  = 0;
				}
			} else {
				drawFontChar(c);
			}
		} else if (gfxFont)  {
			if (c == '\n') {
	            cursor_y += (int16_t)textsize_y * gfxFont->yAdvance;
				if(scrollEnable && isWritingScrollArea){
					cursor_x  = scroll_x;
				}else{
					cursor_x  = 0;
				}
			} else {
				drawGFXFontChar(c);
			}
		} else {
			if (c == '\n') {
				cursor_y += textsize_y*8;
				if(scrollEnable && isWritingScrollArea){
					cursor_x  = scroll_x;
				}else{
					cursor_x  = 0;
				}
			} else if (c == '\r') {
				// skip em
			} else {
				if(scrollEnable && isWritingScrollArea && (cursor_y > (scroll_y+scroll_height - textsize_y*8))){
					scrollTextArea(textsize_y*8);
					cursor_y -= textsize_y*8;
					cursor_x = scroll_x;
				}
				drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x, textsize_y);
				cursor_x += textsize_x*6;
				if(wrap && scrollEnable && isWritingScrollArea && (cursor_x > (scroll_x+scroll_width - textsize_x*6))){
					cursor_y += textsize_y*8;
					cursor_x = scroll_x;
				}
				else if (wrap && (cursor_x > (_width - textsize_x*6))) {
					cursor_y += textsize_y*8;
					cursor_x = 0;
				}
			}
		}
	}
	return size;
}

// Draw a character
void ST7735_t3::drawChar(int16_t x, int16_t y, unsigned char c,
			    uint16_t fgcolor, uint16_t bgcolor, uint8_t size_x, uint8_t size_y)
{
	if((x >= _width)            || // Clip right
	   (y >= _height)           || // Clip bottom
	   ((x + 6 * size_x - 1) < 0) || // Clip left  TODO: is this correct?
	   ((y + 8 * size_y - 1) < 0))   // Clip top   TODO: is this correct?
		return;

	if (fgcolor == bgcolor) {
		// This transparent approach is only about 20% faster
		if ((size_x == 1) && (size_y == 1)) {
			uint8_t mask = 0x01;
			int16_t xoff, yoff;
			for (yoff=0; yoff < 8; yoff++) {
				uint8_t line = 0;
				for (xoff=0; xoff < 5; xoff++) {
					if (glcdfont[c * 5 + xoff] & mask) line |= 1;
					line <<= 1;
				}
				line >>= 1;
				xoff = 0;
				while (line) {
					if (line == 0x1F) {
						drawFastHLine(x + xoff, y + yoff, 5, fgcolor);
						break;
					} else if (line == 0x1E) {
						drawFastHLine(x + xoff, y + yoff, 4, fgcolor);
						break;
					} else if ((line & 0x1C) == 0x1C) {
						drawFastHLine(x + xoff, y + yoff, 3, fgcolor);
						line <<= 4;
						xoff += 4;
					} else if ((line & 0x18) == 0x18) {
						drawFastHLine(x + xoff, y + yoff, 2, fgcolor);
						line <<= 3;
						xoff += 3;
					} else if ((line & 0x10) == 0x10) {
						drawPixel(x + xoff, y + yoff, fgcolor);
						line <<= 2;
						xoff += 2;
					} else {
						line <<= 1;
						xoff += 1;
					}
				}
				mask = mask << 1;
			}
		} else {
			uint8_t mask = 0x01;
			int16_t xoff, yoff;
			for (yoff=0; yoff < 8; yoff++) {
				uint8_t line = 0;
				for (xoff=0; xoff < 5; xoff++) {
					if (glcdfont[c * 5 + xoff] & mask) line |= 1;
					line <<= 1;
				}
				line >>= 1;
				xoff = 0;
				while (line) {
					if (line == 0x1F) {
						fillRect(x + xoff * size_x, y + yoff * size_y,
							5 * size_x, size_y, fgcolor);
						break;
					} else if (line == 0x1E) {
						fillRect(x + xoff * size_x, y + yoff * size_y,
							4 * size_x, size_y, fgcolor);
						break;
					} else if ((line & 0x1C) == 0x1C) {
						fillRect(x + xoff * size_x, y + yoff * size_y,
							3 * size_x, size_y, fgcolor);
						line <<= 4;
						xoff += 4;
					} else if ((line & 0x18) == 0x18) {
						fillRect(x + xoff * size_x, y + yoff * size_y,
							2 * size_x, size_y, fgcolor);
						line <<= 3;
						xoff += 3;
					} else if ((line & 0x10) == 0x10) {
						fillRect(x + xoff * size_x, y + yoff * size_y,
							size_x, size_y, fgcolor);
						line <<= 2;
						xoff += 2;
					} else {
						line <<= 1;
						xoff += 1;
					}
				}
				mask = mask << 1;
			}
		}
	} else {
		// This solid background approach is about 5 time faster
		uint8_t xc, yc;
		uint8_t xr, yr;
		uint8_t mask = 0x01;
		uint16_t color;

		// We need to offset by the origin.
		x+=_originx;
		y+=_originy;
		int16_t x_char_start = x;  // remember our X where we start outputting...

		if((x >= _displayclipx2)            || // Clip right
			 (y >= _displayclipy2)           || // Clip bottom
			 ((x + 6 * size_x - 1) < _displayclipx1) || // Clip left  TODO: this is not correct
			 ((y + 8 * size_y - 1) < _displayclipy1))   // Clip top   TODO: this is not correct
			return;


		#ifdef ENABLE_ST77XX_FRAMEBUFFER
		if (_use_fbtft) {
			uint16_t * pfbPixel_row = &_pfbtft[ y*_width + x];
			for (yc=0; (yc < 8) && (y < _displayclipy2); yc++) {
				for (yr=0; (yr < size_y) && (y < _displayclipy2); yr++) {
					x = x_char_start; 		// get our first x position...
					if (y >= _displayclipy1) {
						uint16_t * pfbPixel = pfbPixel_row;
						for (xc=0; xc < 5; xc++) {
							if (glcdfont[c * 5 + xc] & mask) {
								color = fgcolor;
							} else {
								color = bgcolor;
							}
							for (xr=0; xr < size_x; xr++) {
								if ((x >= _displayclipx1) && (x < _displayclipx2)) {
									*pfbPixel = color;
								}
								pfbPixel++;
								x++;
							}
						}
						for (xr=0; xr < size_x; xr++) {
							if ((x >= _displayclipx1) && (x < _displayclipx2)) {
								*pfbPixel = bgcolor;
							}
							pfbPixel++;
							x++;
						}
					}
					pfbPixel_row += _width; // setup pointer to 
					y++;
				}
				mask = mask << 1;
			}

		} else 
		#endif
		{
			// need to build actual pixel rectangle we will output into.
			int16_t y_char_top = y;	// remember the y
			int16_t w =  6 * size_x;
			int16_t h = 8 * size_y;

			if(x < _displayclipx1) {	w -= (_displayclipx1-x); x = _displayclipx1; 	}
			if((x + w - 1) >= _displayclipx2)  w = _displayclipx2  - x;
			if(y < _displayclipy1) {	h -= (_displayclipy1 - y); y = _displayclipy1; 	}
			if((y + h - 1) >= _displayclipy2) h = _displayclipy2 - y;

			beginSPITransaction();
			setAddr(x, y, x + w -1, y + h - 1);

			y = y_char_top;	// restore the actual y.
			writecommand(ST7735_RAMWR);
			for (yc=0; (yc < 8) && (y < _displayclipy2); yc++) {
				for (yr=0; (yr < size_y) && (y < _displayclipy2); yr++) {
					x = x_char_start; 		// get our first x position...
					if (y >= _displayclipy1) {
						for (xc=0; xc < 5; xc++) {
							if (glcdfont[c * 5 + xc] & mask) {
								color = fgcolor;
							} else {
								color = bgcolor;
							}
							for (xr=0; xr < size_x; xr++) {
								if ((x >= _displayclipx1) && (x < _displayclipx2)) {
									writedata16(color);
								}
								x++;
							}
						}
						for (xr=0; xr < size_x; xr++) {
							if ((x >= _displayclipx1) && (x < _displayclipx2)) {
								writedata16(bgcolor);
							}
							x++;
						}
					}
					y++;
				}
				mask = mask << 1;
			}
			writecommand_last(ST7735_NOP);
			endSPITransaction();
		}
	}
}

void ST7735_t3::setFont(const ILI9341_t3_font_t &f) {
	font = &f;
	_gfx_last_char_x_write = 0;	// Don't use cached data here
	if (gfxFont) {
        cursor_y -= 6;
		gfxFont = NULL;
	}
	fontbpp = 1;
	// Calculate additional metrics for Anti-Aliased font support (BDF extn v2.3)
	if (font && font->version==23){
		fontbpp = (font->reserved & 0b000011)+1;
		fontbppindex = (fontbpp >> 2)+1;
		fontbppmask = (1 << (fontbppindex+1))-1;
		fontppb = 8/fontbpp;
		fontalphamx = 31/((1<<fontbpp)-1);
		// Ensure text and bg color are different. Note: use setTextColor to set actual bg color
		if (textcolor == textbgcolor) textbgcolor = (textcolor==0x0000)?0xFFFF:0x0000;
	}
}

// Maybe support GFX Fonts as well?
void ST7735_t3::setFont(const GFXfont *f) {
	font = NULL;	// turn off the other font... 
	_gfx_last_char_x_write = 0;	// Don't use cached data here
	if (f == gfxFont) return;	// same font or lack of so can bail.

    if(f) {            // Font struct pointer passed in?
        if(!gfxFont) { // And no current font struct?
            // Switching from classic to new font behavior.
            // Move cursor pos down 6 pixels so it's on baseline.
            cursor_y += 6;
        }

        // Test wondering high and low of Ys here... 
        int8_t miny_offset = 0;
#if 1
        for (uint8_t i=0; i <= (f->last - f->first); i++) {
        	if (f->glyph[i].yOffset < miny_offset) {
        		miny_offset = f->glyph[i].yOffset;
        	}
        }
#else        
        int max_delta = 0;
        uint8_t index_min = 0;
        uint8_t index_max = 0;

        int8_t minx_offset = 127;
        int8_t maxx_overlap = 0;
        uint8_t indexx_min = 0;
        uint8_t indexx_max = 0;
        for (uint8_t i=0; i <= (f->last - f->first); i++) {
        	if (f->glyph[i].yOffset < miny_offset) {
        		miny_offset = f->glyph[i].yOffset;
        		index_min = i;
        	}

        	if (f->glyph[i].xOffset < minx_offset) {
        		minx_offset = f->glyph[i].xOffset;
        		indexx_min = i;
        	}
        	if ( (f->glyph[i].yOffset + f->glyph[i].height) > max_delta) {
        		max_delta = (f->glyph[i].yOffset + f->glyph[i].height);
        		index_max = i;
        	}
        	int8_t x_overlap = f->glyph[i].xOffset + f->glyph[i].width - f->glyph[i].xAdvance;
        	if (x_overlap > maxx_overlap) {
        		maxx_overlap = x_overlap;
        		indexx_max = i;
        	}
        }
        Serial.printf("Set GFX Font(%x): Y: %d %d(%c) %d(%c) X: %d(%c) %d(%c)\n", (uint32_t)f, f->yAdvance, 
        	miny_offset, index_min + f->first, max_delta, index_max + f->first,
        	minx_offset, indexx_min + f->first, maxx_overlap, indexx_max + f->first);
#endif
        _gfxFont_min_yOffset = miny_offset;	// Probably only thing we need... May cache? 

    } else if(gfxFont) { // NULL passed.  Current font struct defined?
        // Switching from new to classic font behavior.
        // Move cursor pos up 6 pixels so it's at top-left of char.
        cursor_y -= 6;
    }
    gfxFont = f;
}

static uint32_t fetchbit(const uint8_t *p, uint32_t index)
{
	if (p[index >> 3] & (1 << (7 - (index & 7)))) return 1;
	return 0;
}

static uint32_t fetchbits_unsigned(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = 0;
	do {
		uint8_t b = p[index >> 3];
		uint32_t avail = 8 - (index & 7);
		if (avail <= required) {
			val <<= avail;
			val |= b & ((1 << avail) - 1);
			index += avail;
			required -= avail;
		} else {
			b >>= avail - required;
			val <<= required;
			val |= b & ((1 << required) - 1);
			break;
		}
	} while (required);
	return val;
}

static uint32_t fetchbits_signed(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = fetchbits_unsigned(p, index, required);
	if (val & (1 << (required - 1))) {
		return (int32_t)val - (1 << required);
	}
	return (int32_t)val;
}

uint32_t ST7735_t3::fetchpixel(const uint8_t *p, uint32_t index, uint32_t x)
{
	// The byte
	uint8_t b = p[index >> 3];
	// Shift to LSB position and mask to get value
	uint8_t s = ((fontppb-(x % fontppb)-1)*fontbpp);
	// Mask and return
	return (b >> s) & fontbppmask;
}

void ST7735_t3::drawFontChar(unsigned int c)
{
	uint32_t bitoffset;
	const uint8_t *data;

	//Serial.printf("drawFontChar(%c) %d\n", c, c);

	if (c >= font->index1_first && c <= font->index1_last) {
		bitoffset = c - font->index1_first;
		bitoffset *= font->bits_index;
	} else if (c >= font->index2_first && c <= font->index2_last) {
		bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
		bitoffset *= font->bits_index;
	} else if (font->unicode) {
		return; // TODO: implement sparse unicode
	} else {
		return;
	}
	//Serial.printf("  index =  %d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
	data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

	uint32_t encoding = fetchbits_unsigned(data, 0, 3);
	if (encoding != 0) return;
	uint32_t width = fetchbits_unsigned(data, 3, font->bits_width);
	bitoffset = font->bits_width + 3;
	uint32_t height = fetchbits_unsigned(data, bitoffset, font->bits_height);
	bitoffset += font->bits_height;
	//Serial.printf("  size =   %d,%d\n", width, height);
	//Serial.printf("  line space = %d\n", font->line_space);


	int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
	bitoffset += font->bits_xoffset;
	int32_t yoffset = fetchbits_signed(data, bitoffset, font->bits_yoffset);
	bitoffset += font->bits_yoffset;
	//Serial.printf("  offset = %d,%d\n", xoffset, yoffset);
    //Serial.printf("DChar: %c %u, %u, wh:%d %d o:%d %d\n", c, cursor_x, cursor_y, width, height, xoffset, yoffset);

	uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
	bitoffset += font->bits_delta;
	//Serial.printf("  delta =  %d\n", delta);

	//Serial.printf("  cursor = %d,%d\n", cursor_x, cursor_y);

	 //horizontally, we draw every pixel, or none at all
	if (cursor_x < 0) cursor_x = 0;
	int32_t origin_x = cursor_x + xoffset;
	if (origin_x < 0) {
		cursor_x -= xoffset;
		origin_x = 0;
	}
	if (origin_x + (int)width > _width) {
		if (!wrap) return;
		origin_x = 0;
		if (xoffset >= 0) {
			cursor_x = 0;
		} else {
			cursor_x = -xoffset;
		}
		cursor_y += font->line_space;
	}
	if(wrap && scrollEnable && isWritingScrollArea && ((origin_x + (int)width) > (scroll_x+scroll_width))){
    	origin_x = 0;
		if (xoffset >= 0) {
			cursor_x = scroll_x;
		} else {
			cursor_x = -xoffset;
		}
		cursor_y += font->line_space;
    }
	
	if(scrollEnable && isWritingScrollArea && (cursor_y > (scroll_y+scroll_height - font->cap_height))){
		scrollTextArea(font->line_space);
		cursor_y -= font->line_space;
		cursor_x = scroll_x;
	} 
	if (cursor_y >= _height) return;

	// vertically, the top and/or bottom can be clipped
	int32_t origin_y = cursor_y + font->cap_height - height - yoffset;
	//Serial.printf("  origin = %d,%d\n", origin_x, origin_y);

	// TODO: compute top skip and number of lines
	int32_t linecount = height;
	//uint32_t loopcount = 0;
	int32_t y = origin_y;
	bool opaque = (textbgcolor != textcolor);

	// Going to try a fast Opaque method which works similar to drawChar, which is near the speed of writerect
	if (!opaque) {

		// Anti-alias support
		if (fontbpp>1){
			// This branch should, in most cases, never happen. This is because if an
			// anti-aliased font is being used, textcolor and textbgcolor should always
			// be different. Even though an anti-alised font is being used, pixels in this
			// case will all be solid because pixels are rendered on same colour as themselves!
			// This won't look very good.
			bitoffset = ((bitoffset + 7) & (-8)); // byte-boundary
			uint32_t xp = 0;
			uint8_t halfalpha = 1<<(fontbpp-1);
			while (linecount) {
				uint32_t x = 0;
				while(x<width) {
					// One pixel at a time, either on (if alpha > 0.5) or off
					if (fetchpixel(data, bitoffset, xp)>=halfalpha){
						Pixel(origin_x + x,y,textcolor);
					}
					bitoffset += fontbpp;
					x++;
					xp++;
				}
				y++;
				linecount--;
			}

		}
		// Soild pixels
		else{

			while (linecount > 0) {
				//Serial.printf("    linecount = %d\n", linecount);
				uint32_t n = 1;
				if (fetchbit(data, bitoffset++) != 0) {
					n = fetchbits_unsigned(data, bitoffset, 3) + 2;
					bitoffset += 3;
				}
				uint32_t x = 0;
				do {
					int32_t xsize = width - x;
					if (xsize > 32) xsize = 32;
					uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
					//Serial.printf("    multi line %d %d %x\n", n, x, bits);
					drawFontBits(opaque, bits, xsize, origin_x + x, y, n);
					bitoffset += xsize;
					x += xsize;
				} while (x < width);


				y += n;
				linecount -= n;
				//if (++loopcount > 100) {
					//Serial.println("     abort draw loop");
					//break;
				//}
			}
		} // 1bpp
	}

	// opaque
	else {
		// Now opaque mode... 
		// Now write out background color for the number of rows above the above the character
		// figure out bounding rectangle... 
		// In this mode we need to update to use the offset and bounding rectangles as we are doing it it direct.
		// also update the Origin 
		int cursor_x_origin = cursor_x + _originx;
		int cursor_y_origin = cursor_y + _originy;
		origin_x += _originx;
		origin_y += _originy;

		int start_x = (origin_x < cursor_x_origin) ? origin_x : cursor_x_origin; 	
		if (start_x < 0) start_x = 0;
		
		int start_y = (origin_y < cursor_y_origin) ? origin_y : cursor_y_origin; 
		if (start_y < 0) start_y = 0;
		int end_x = cursor_x_origin + delta; 
		if ((origin_x + (int)width) > end_x)
			end_x = origin_x + (int)width;
		if (end_x >= _displayclipx2)  end_x = _displayclipx2;	
		int end_y = cursor_y_origin + font->line_space; 
		if ((origin_y + (int)height) > end_y)
			end_y = origin_y + (int)height;
		if (end_y >= _displayclipy2) end_y = _displayclipy2;	
		end_x--;	// setup to last one we draw
		end_y--;
		int start_x_min = (start_x >= _displayclipx1) ? start_x : _displayclipx1;
		int start_y_min = (start_y >= _displayclipy1) ? start_y : _displayclipy1;

		// See if anything is in the display area.
		if((end_x < _displayclipx1) ||(start_x >= _displayclipx2) || (end_y < _displayclipy1) || (start_y >= _displayclipy2)) {
			cursor_x += delta;	// could use goto or another indent level...
		 	return;
		}
/*
		Serial.printf("drawFontChar(%c) %d\n", c, c);
		Serial.printf("  size =   %d,%d\n", width, height);
		Serial.printf("  line space = %d\n", font->line_space);
		Serial.printf("  offset = %d,%d\n", xoffset, yoffset);
		Serial.printf("  delta =  %d\n", delta);
		Serial.printf("  cursor = %d,%d\n", cursor_x, cursor_y);
		Serial.printf("  origin = %d,%d\n", origin_x, origin_y);

		Serial.printf("  Bounding: (%d, %d)-(%d, %d)\n", start_x, start_y, end_x, end_y);
		Serial.printf("  mins (%d %d),\n", start_x_min, start_y_min);
*/
		#ifdef ENABLE_ST77XX_FRAMEBUFFER
		if (_use_fbtft) {
			uint16_t * pfbPixel_row = &_pfbtft[ start_y*_width + start_x];
			uint16_t * pfbPixel;
			int screen_y = start_y;
			int screen_x;

			// Clear above character
			while (screen_y < origin_y) {
				pfbPixel = pfbPixel_row;
				// only output if this line is within the clipping region.
				if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
					for (screen_x = start_x; screen_x <= end_x; screen_x++) {
						if (screen_x >= _displayclipx1) {
							*pfbPixel = textbgcolor;
						}
						pfbPixel++;
					}
				}
				screen_y++;
				pfbPixel_row += _width;
			}

			// Anti-aliased font
			if (fontbpp>1){
				screen_y = origin_y;
				bitoffset = ((bitoffset + 7) & (-8)); // byte-boundary
				uint32_t xp = 0;
				int glyphend_x = origin_x+width;
				while (linecount) {
					pfbPixel = pfbPixel_row;
					screen_x = start_x;
					while(screen_x<=end_x) {
						// XXX: I'm sure clipping could be done way more efficiently than just chekcing every single pixel, but let's just get this going
						if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2) && (screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							// Clear before or after pixel
							if ((screen_x<origin_x) || (screen_x>=glyphend_x)){
								*pfbPixel = textbgcolor;
							}
							// Draw alpha-blended character
							else{
								uint8_t alpha = fetchpixel(data, bitoffset, xp);
								*pfbPixel = alphaBlendRGB565Premultiplied( textcolorPrexpanded, textbgcolorPrexpanded, (uint8_t)(alpha * fontalphamx) );
								bitoffset += fontbpp;
								xp++;
							}
						} // clip
						screen_x++;
						pfbPixel++;
					}
					pfbPixel_row += _width;
					screen_y++;
					linecount--;
				}

			} // anti-aliased

			// 1bpp solid font
			else{

				// Now lets process each of the data lines (draw character)
				screen_y = origin_y;
				while (linecount > 0) {
					//Serial.printf("    linecount = %d\n", linecount);
					uint32_t b = fetchbit(data, bitoffset++);
					uint32_t n;
					if (b == 0) {
						//Serial.println("Single");
						n = 1;
					} else {
						//Serial.println("Multi");
						n = fetchbits_unsigned(data, bitoffset, 3) + 2;
						bitoffset += 3;
					}
					uint32_t bitoffset_row_start = bitoffset;
					while (n--) {
						pfbPixel = pfbPixel_row;

						// Clear to left
						if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							bitoffset = bitoffset_row_start;	// we will work through these bits maybe multiple times
							for (screen_x = start_x; screen_x < origin_x; screen_x++) {
								if (screen_x >= _displayclipx1) {
									*pfbPixel = textbgcolor;
								} // make sure not clipped
								pfbPixel++;
							}
						}

						// Pixel bits
						screen_x = origin_x;
						uint32_t x = 0;
						do {
							uint32_t xsize = width - x;
							if (xsize > 32) xsize = 32;
							uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
							uint32_t bit_mask = 1 << (xsize-1);
							//Serial.printf(" %d %d %x %x\n", x, xsize, bits, bit_mask);
							if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
								while (bit_mask && (screen_x <= end_x)) {
									if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2)) {
										*pfbPixel = (bits & bit_mask) ? textcolor : textbgcolor;
									}
									pfbPixel++;	
									bit_mask = bit_mask >> 1;
									screen_x++;	// increment our pixel position. 
								}
							}
								bitoffset += xsize;
							x += xsize;
						} while (x < width);

						// Clear to right
						if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							// output bg color and right hand side
							while (screen_x++ <= end_x) {
								*pfbPixel++ = textbgcolor;
							}
						}			 
			 			screen_y++;
						pfbPixel_row += _width;
						linecount--;
					}
				}

			} // 1bpp

			// clear below character
	 		while (screen_y++ <= end_y) {
				if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
					pfbPixel = pfbPixel_row;
					for (screen_x = start_x; screen_x <= end_x; screen_x++) {
						if (screen_x >= _displayclipx1) {
							*pfbPixel = textbgcolor;
						}
						pfbPixel++;
					}
				}
				pfbPixel_row += _width;
			}

		} else 
		#endif
		{

			beginSPITransaction();
			//Serial.printf("SetAddr %d %d %d %d\n", start_x_min, start_y_min, end_x, end_y);
			// output rectangle we are updating... We have already clipped end_x/y, but not yet start_x/y
			setAddr( start_x, start_y_min, end_x, end_y);
			writecommand(ST7735_RAMWR);
			int screen_y = start_y_min;
			int screen_x;

			// Clear above character
			while (screen_y < origin_y) {
				for (screen_x = start_x_min; screen_x <= end_x; screen_x++) {
					writedata16(textbgcolor);
				}
				screen_y++;
			}

			// Anti-aliased font
			if (fontbpp>1){
				screen_y = origin_y;
				bitoffset = ((bitoffset + 7) & (-8)); // byte-boundary
				int glyphend_x = origin_x+width;
				uint32_t xp = 0;
				while (linecount) {
				 	screen_x = start_x;
					while(screen_x<=end_x) {
						// XXX: I'm sure clipping could be done way more efficiently than just chekcing every single pixel, but let's just get this going
						if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2) && (screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							// Clear before or after pixel
							if ((screen_x<origin_x) || (screen_x>=glyphend_x)){
								writedata16(textbgcolor);
							}
							// Draw alpha-blended character
							else{
								uint8_t alpha = fetchpixel(data, bitoffset, xp);
								writedata16( alphaBlendRGB565Premultiplied( textcolorPrexpanded, textbgcolorPrexpanded, (uint8_t)(alpha * fontalphamx) ) );
								bitoffset += fontbpp;
								xp++;
							}
						} // clip
						screen_x++;
					}
					screen_y++;
					linecount--;
				}

			} // anti-aliased

			// 1bpp
			else{

				// Now lets process each of the data lines. 
				screen_y = origin_y;
				while (linecount > 0) {
					//Serial.printf("    linecount = %d\n", linecount);
					uint32_t b = fetchbit(data, bitoffset++);
					uint32_t n;
					if (b == 0) {
						//Serial.println("    Single");
						n = 1;
					} else {
						//Serial.println("    Multi");
						n = fetchbits_unsigned(data, bitoffset, 3) + 2;
						bitoffset += 3;
					}
					uint32_t bitoffset_row_start = bitoffset;
					while (n--) {
						// do some clipping here. 
						bitoffset = bitoffset_row_start;	// we will work through these bits maybe multiple times
						// We need to handle case where some of the bits may not be visible, but we still need to
						// read through them
						//Serial.printf("y:%d  %d %d %d %d\n", screen_y, start_x, origin_x, _displayclipx1, _displayclipx2);
						if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							for (screen_x = start_x; screen_x < origin_x; screen_x++) {
								if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2)) {
									//Serial.write('-');
									writedata16(textbgcolor);
								}
							}
						}	
						uint32_t x = 0;
						screen_x = origin_x;
						do {
							uint32_t xsize = width - x;
							if (xsize > 32) xsize = 32;
							uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
							uint32_t bit_mask = 1 << (xsize-1);
							//Serial.printf("     %d %d %x %x - ", x, xsize, bits, bit_mask);
							if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
								while (bit_mask) {
									if ((screen_x >= _displayclipx1) && (screen_x < _displayclipx2)) {
										writedata16((bits & bit_mask) ? textcolor : textbgcolor);
										//Serial.write((bits & bit_mask) ? '*' : '.');
									}
									bit_mask = bit_mask >> 1;
									screen_x++ ; // Current actual screen X
								}
								//Serial.println();
								bitoffset += xsize;
							}
							x += xsize;
						} while (x < width) ;
						if ((screen_y >= _displayclipy1) && (screen_y < _displayclipy2)) {
							// output bg color and right hand side
							while (screen_x++ <= end_x) {
								writedata16(textbgcolor);
								//Serial.write('+');
							}
							//Serial.println();
						}
			 			screen_y++;
						linecount--;
					}
				}
			} // 1bpp

			// clear below character - note reusing xcreen_x for this
			screen_x = (end_y + 1 - screen_y) * (end_x + 1 - start_x_min); // How many bytes we need to still output
			//Serial.printf("Clear Below: %d\n", screen_x);
			while (screen_x-- > 1) {
				writedata16(textbgcolor);
			}
			writedata16_last(textbgcolor);
			endSPITransaction();
		}

	}
	// Increment to setup for the next character.
	cursor_x += delta;

}

//strPixelLen			- gets pixel length of given ASCII string
int16_t ST7735_t3::strPixelLen(const char * str)
{
//	//Serial.printf("strPixelLen %s\n", str);
	if (!str) return(0);
	if (gfxFont) 
	{
		// BUGBUG:: just use the other function for now... May do this for all of them...
	  int16_t x, y;
	  uint16_t w, h;
	  getTextBounds(str, cursor_x, cursor_y, &x, &y, &w, &h);
	  return w;
	}

	uint16_t len=0, maxlen=0;
	while (*str)
	{
		if (*str=='\n')
		{
			if ( len > maxlen )
			{
				maxlen=len;
				len=0;
			}
		}
		else
		{
			if (!font)
			{
				len+=textsize_x*6;
			}
			else
			{

				uint32_t bitoffset;
				const uint8_t *data;
				uint16_t c = *str;

//				//Serial.printf("char %c(%d)\n", c,c);

				if (c >= font->index1_first && c <= font->index1_last) {
					bitoffset = c - font->index1_first;
					bitoffset *= font->bits_index;
				} else if (c >= font->index2_first && c <= font->index2_last) {
					bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
					bitoffset *= font->bits_index;
				} else if (font->unicode) {
					continue;
				} else {
					continue;
				}
				//Serial.printf("  index =  %d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
				data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

				uint32_t encoding = fetchbits_unsigned(data, 0, 3);
				if (encoding != 0) continue;
//				uint32_t width = fetchbits_unsigned(data, 3, font->bits_screenWidth);
//				//Serial.printf("  width =  %d\n", width);
				bitoffset = font->bits_width + 3;
				bitoffset += font->bits_height;

//				int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
//				//Serial.printf("  xoffset =  %d\n", xoffset);
				bitoffset += font->bits_xoffset;
				bitoffset += font->bits_yoffset;

				uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
				bitoffset += font->bits_delta;
//				//Serial.printf("  delta =  %d\n", delta);

				len += delta;//+width-xoffset;
//				//Serial.printf("  len =  %d\n", len);
			}

			if ( len > maxlen )
			{
				maxlen=len;
//					//Serial.printf("  maxlen =  %d\n", maxlen);
			}
		}
		str++;
	}
//	//Serial.printf("Return  maxlen =  %d\n", maxlen);
	return( maxlen );
}

void ST7735_t3::charBounds(char c, int16_t *x, int16_t *y,
  int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy) {

	// BUGBUG:: Not handling offset/clip
    if (font) {
        if(c == '\n') { // Newline?
            *x  = 0;    // Reset x to zero, advance y by one line
            *y += font->line_space;
        } else if(c != '\r') { // Not a carriage return; is normal char
			uint32_t bitoffset;
			const uint8_t *data;
			if (c >= font->index1_first && c <= font->index1_last) {
				bitoffset = c - font->index1_first;
				bitoffset *= font->bits_index;
			} else if (c >= font->index2_first && c <= font->index2_last) {
				bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
				bitoffset *= font->bits_index;
			} else if (font->unicode) {
				return; // TODO: implement sparse unicode
			} else {
				return;
			}
			//Serial.printf("  index =  %d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
			data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

			uint32_t encoding = fetchbits_unsigned(data, 0, 3);
			if (encoding != 0) return;
			uint32_t width = fetchbits_unsigned(data, 3, font->bits_width);
			bitoffset = font->bits_width + 3;
			uint32_t height = fetchbits_unsigned(data, bitoffset, font->bits_height);
			bitoffset += font->bits_height;
			//Serial.printf("  size =   %d,%d\n", width, height);
			//Serial.printf("  line space = %d\n", font->line_space);

			int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
			bitoffset += font->bits_xoffset;
			int32_t yoffset = fetchbits_signed(data, bitoffset, font->bits_yoffset);
			bitoffset += font->bits_yoffset;

			uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
			bitoffset += font->bits_delta;

            int16_t
                    x1 = *x + xoffset,
                    y1 = *y + font->cap_height - height - yoffset,
                    x2 = x1 + width,
                    y2 = y1 + height;

		    if(wrap && (x2 > _width)) {
	            *x  = 0; // Reset x to zero, advance y by one line
	            *y += font->line_space;
	            x1 = *x + xoffset,
	            y1 = *y + yoffset,
	            x2 = x1 + width,
	            y2 = y1 + height;
        	}
            if(x1 < *minx) *minx = x1;
            if(y1 < *miny) *miny = y1;
            if(x2 > *maxx) *maxx = x2;
            if(y2 > *maxy) *maxy = y2;
            *x += delta;	// ? guessing here...
        }
    } 

    else if(gfxFont) {

        if(c == '\n') { // Newline?
            *x  = 0;    // Reset x to zero, advance y by one line
            *y += textsize_y * gfxFont->yAdvance;
        } else if(c != '\r') { // Not a carriage return; is normal char
            uint8_t first = gfxFont->first,
                    last  = gfxFont->last;
            if((c >= first) && (c <= last)) { // Char present in this font?
    			GFXglyph *glyph  = gfxFont->glyph + (c - first);
                uint8_t gw = glyph->width,
                        gh = glyph->height,
                        xa = glyph->xAdvance;
                int8_t  xo = glyph->xOffset,
                        yo = glyph->yOffset + gfxFont->yAdvance/2;
                if(wrap && ((*x+(((int16_t)xo+gw)*textsize_x)) > _width)) {
                    *x  = 0; // Reset x to zero, advance y by one line
                    *y += textsize_y * gfxFont->yAdvance;
                }
                int16_t tsx = (int16_t)textsize_x,
                        tsy = (int16_t)textsize_y,
                        x1 = *x + xo * tsx,
                        y1 = *y + yo * tsy,
                        x2 = x1 + gw * tsx - 1,
                        y2 = y1 + gh * tsy - 1;
                if(x1 < *minx) *minx = x1;
                if(y1 < *miny) *miny = y1;
                if(x2 > *maxx) *maxx = x2;
                if(y2 > *maxy) *maxy = y2;
                *x += xa * tsx;
            }
        }

    } else { // Default font

        if(c == '\n') {                     // Newline?
            *x  = 0;                        // Reset x to zero,
            *y += textsize_y * 8;           // advance y one line
            // min/max x/y unchaged -- that waits for next 'normal' character
        } else if(c != '\r') {  // Normal char; ignore carriage returns
            if(wrap && ((*x + textsize_x * 6) > _width)) { // Off right?
                *x  = 0;                    // Reset x to zero,
                *y += textsize_y * 8;       // advance y one line
            }
            int x2 = *x + textsize_x * 6 - 1, // Lower-right pixel of char
                y2 = *y + textsize_y * 8 - 1;
            if(x2 > *maxx) *maxx = x2;      // Track max x, y
            if(y2 > *maxy) *maxy = y2;
            if(*x < *minx) *minx = *x;      // Track min x, y
            if(*y < *miny) *miny = *y;
            *x += textsize_x * 6;             // Advance x one char
        }
    }
}

// Add in Adafruit versions of text bounds calculations. 
void ST7735_t3::getTextBounds(const uint8_t *buffer, uint16_t len, int16_t x, int16_t y,
      int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
    *x1 = x;
    *y1 = y;
    *w  = *h = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while(len--)
        charBounds(*buffer++, &x, &y, &minx, &miny, &maxx, &maxy);

    if(maxx >= minx) {
        *x1 = minx;
        *w  = maxx - minx + 1;
    }
    if(maxy >= miny) {
        *y1 = miny;
        *h  = maxy - miny + 1;
    }
}
void ST7735_t3::getTextBounds(const char *str, int16_t x, int16_t y,
        int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
    uint8_t c; // Current character

    *x1 = x;
    *y1 = y;
    *w  = *h = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while((c = *str++))
        charBounds(c, &x, &y, &minx, &miny, &maxx, &maxy);

    if(maxx >= minx) {
        *x1 = minx;
        *w  = maxx - minx + 1;
    }
    if(maxy >= miny) {
        *y1 = miny;
        *h  = maxy - miny + 1;
    }
}

void ST7735_t3::getTextBounds(const String &str, int16_t x, int16_t y,
        int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
    if (str.length() != 0) {
        getTextBounds(const_cast<char*>(str.c_str()), x, y, x1, y1, w, h);
    }
}



void ST7735_t3::drawFontPixel( uint8_t alpha, uint32_t x, uint32_t y ){
	// Adjust alpha based on the number of alpha levels supported by the font (based on bpp)
	// Note: Implemented look-up table for alpha, but made absolutely no difference in speed (T3.6)
	alpha = (uint8_t)(alpha * fontalphamx);
	uint32_t result = ((((textcolorPrexpanded - textbgcolorPrexpanded) * alpha) >> 5) + textbgcolorPrexpanded) & 0b00000111111000001111100000011111;
	Pixel(x,y,(uint16_t)((result >> 16) | result));
}

void ST7735_t3::drawFontBits(bool opaque, uint32_t bits, uint32_t numbits, int32_t x, int32_t y, uint32_t repeat)
{
	if (bits == 0) {
		if (opaque) {
			fillRect(x, y, numbits, repeat, textbgcolor);
		}
	} else {
		int32_t x1 = x;
		uint32_t n = numbits;
		int w;
		int bgw;

		w = 0;
		bgw = 0;

		do {
			n--;
			if (bits & (1 << n)) {
				if (bgw>0) {
					if (opaque) {
						fillRect(x1 - bgw, y, bgw, repeat, textbgcolor);
					}
					bgw=0;
				}
				w++;
			} else {
				if (w>0) {
					fillRect(x1 - w, y, w, repeat, textcolor);
					w = 0;
				}
				bgw++;
			}
			x1++;
		} while (n > 0);

		if (w > 0) {
			fillRect(x1 - w, y, w, repeat, textcolor);
		}

		if (bgw > 0) {
			if (opaque) {
				fillRect(x1 - bgw, y, bgw, repeat, textbgcolor);
			}
		}
	}
}

void ST7735_t3::drawGFXFontChar(unsigned int c) {
	// Lets do Adafruit GFX character output here as well
    if(c == '\r') 	 return;

    // Some quick and dirty tests to see if we can
    uint8_t first = gfxFont->first;
    if((c < first) || (c > gfxFont->last)) return; 

    GFXglyph *glyph  = gfxFont->glyph + (c - first);
    uint8_t   w     = glyph->width,
              h     = glyph->height;
			  
    // wonder if we should look at xo, yo instead?         
    if((w == 0 ||  h == 0)  && (c != 32))   return;  // Is there an associated bitmap?
	
    int16_t xo = glyph->xOffset; // sic
    int16_t yo = glyph->yOffset + gfxFont->yAdvance/2;

    if(wrap && ((cursor_x + textsize_x * (xo + w)) > _width)) {
        cursor_x  = 0;
        cursor_y += (int16_t)textsize_y * gfxFont->yAdvance;
    }

    // Lets do the work to output the font character
    uint8_t  *bitmap = gfxFont->bitmap;

    uint16_t bo = glyph->bitmapOffset;
    uint8_t  xx, yy, bits = 0, bit = 0;
    //Serial.printf("DGFX_char: %c (%d,%d) : %u %u %u %u %d %d %x %x %d\n", c, cursor_x, cursor_y, w, h,  
    //			glyph->xAdvance, gfxFont->yAdvance, xo, yo, textcolor, textbgcolor, _use_fbtft);Serial.flush();

    if (textcolor == textbgcolor) {

	     //Serial.printf("DGFXChar: %c %u, %u, wh:%d %d o:%d %d\n", c, cursor_x, cursor_y, w, h, xo, yo);

    	// NOTE: Adafruit GFX does not support Opaque font output as there
    	// are issues with proportionally spaced characters that may overlap
    	// So the below is not perfect as we may overwrite a small portion 
    	// of a letter with the next one, when we blank out... 
    	// But: I prefer to let each of us decide if the limitations are
    	// worth it or not.  If Not you still have the option to not
    	// Do transparent mode and instead blank out and blink...
	    for(yy=0; yy<h; yy++) {
	    	uint8_t w_left = w;
	    	xx = 0;
	        while (w_left) {
	            if(!(bit & 7)) {
	                bits = bitmap[bo++];
	            }
	            // Could try up to 8 bits at time, but start off trying up to 4
	            uint8_t xCount;
	            if ((w_left >= 8) && ((bits & 0xff) == 0xff)) {
	            	xCount = 8;
	            	//Serial.print("8");
	                fillRect(cursor_x+(xo+xx)*textsize_x, cursor_y+(yo+yy)*textsize_y,
	                     xCount * textsize_x, textsize_y, textcolor);
	            } else if ((w_left >= 4) && ((bits & 0xf0) == 0xf0)) {
	            	xCount = 4;
	            	//Serial.print("4");
	                fillRect(cursor_x+(xo+xx)*textsize_x, cursor_y+(yo+yy)*textsize_y,
	                      xCount * textsize_x, textsize_y, textcolor);
	            } else if ((w_left >= 3) && ((bits & 0xe0) == 0xe0)) {
	            	//Serial.print("3");
	            	xCount = 3;
	                fillRect(cursor_x+(xo+xx)*textsize_x, cursor_y+(yo+yy)*textsize_y,
	                      xCount * textsize_x, textsize_y, textcolor);
	            } else if ((w_left >= 2) && ((bits & 0xc0) == 0xc0)) {
	            	//Serial.print("2");
	            	xCount = 2;
	                fillRect(cursor_x+(xo+xx)*textsize_x, cursor_y+(yo+yy)*textsize_y,
	                      xCount * textsize_x, textsize_y, textcolor);
	            } else {
	            	xCount = 1;
	            	if(bits & 0x80) {
		                if((textsize_x == 1) && (textsize_y == 1)){
		                    drawPixel(cursor_x+xo+xx, cursor_y+yo+yy, textcolor);
		                } else {
		                fillRect(cursor_x+(xo+xx)*textsize_x, cursor_y+(yo+yy)*textsize_y,
		                      textsize_x, textsize_y, textcolor);
		                }
		            }
	            }
	            xx += xCount;
	            w_left -= xCount;
	            bit += xCount;
	            bits <<= xCount;
	        }

	    }
    	_gfx_last_char_x_write = 0;
	} else {
		// To Do, properly clipping and offsetting...
		// This solid background approach is about 5 time faster
		// Lets calculate bounding rectangle that we will update
		// We need to offset by the origin.

		// We are going direct so do some offsets and clipping
		int16_t x_offset_cursor = cursor_x + _originx;	// This is where the offseted cursor is.
		int16_t x_start = x_offset_cursor;  // I am assuming no negative x offsets.
		int16_t x_end = x_offset_cursor + (glyph->xAdvance * textsize_x);
		if (glyph->xAdvance < (xo + w)) x_end = x_offset_cursor + ((xo + w)* textsize_x);  // BUGBUG Overlflows into next char position.
		int16_t x_left_fill = x_offset_cursor + xo * textsize_x;
		int16_t x;

		if (xo < 0) { 
			// Unusual character that goes back into previous character
			//Serial.printf("GFX Font char XO < 0: %c %d %d %u %u %u\n", c, xo, yo, w, h, glyph->xAdvance );
			x_start += xo * textsize_x;
			x_left_fill = 0;	// Don't need to fill anything here... 
		}

		int16_t y_start = cursor_y + _originy + (_gfxFont_min_yOffset * textsize_y)+ gfxFont->yAdvance*textsize_y/2;  // UP to most negative value.
		int16_t y_end = y_start +  gfxFont->yAdvance * textsize_y;  // how far we will update
		int16_t y = y_start;
		//int8_t y_top_fill = (yo - _gfxFont_min_yOffset) * textsize_y;	 // both negative like -10 - -16 = 6...
		int8_t y_top_fill = (yo - gfxFont->yAdvance/2 - _gfxFont_min_yOffset) * textsize_y;

		// See if anything is within clip rectangle, if not bail
		if((x_start >= _displayclipx2)   || // Clip right
			 (y_start >= _displayclipy2) || // Clip bottom
			 (x_end < _displayclipx1)    || // Clip left
			 (y_end < _displayclipy1))  	// Clip top 
		{
			// But remember to first update the cursor position
			cursor_x += glyph->xAdvance * (int16_t)textsize_x;
			return;
		}

		// If our y_end > _displayclipy2 set it to _displayclipy2 as to not have to test both  Likewise X
		if (y_end > _displayclipy2) y_end = _displayclipy2;
		if (x_end > _displayclipx2) x_end = _displayclipx2;

		// If we get here and 
		if (_gfx_last_cursor_y != (cursor_y + _originy))  _gfx_last_char_x_write = 0;

		#ifdef ENABLE_ST77XX_FRAMEBUFFER
		if (_use_fbtft) {
			// lets try to output the values directly...
			uint16_t * pfbPixel_row = &_pfbtft[ y_start *_width + x_start];
			uint16_t * pfbPixel;
			// First lets fill in the top parts above the actual rectangle...
			while (y_top_fill--) {
				pfbPixel = pfbPixel_row;
				if ( (y >= _displayclipy1) && (y < _displayclipy2)) {
					for (int16_t xx = x_start; xx < x_end; xx++) {
						if ((xx >= _displayclipx1) && (xx >= x_offset_cursor)) {
							if ((xx >= _gfx_last_char_x_write) || (*pfbPixel != _gfx_last_char_textcolor))
								*pfbPixel = textbgcolor;
						}
						pfbPixel++;
					}					
				}
				pfbPixel_row += _width;
				y++;
			}
			// Now lets output all of the pixels for each of the rows.. 
			for(yy=0; (yy<h) && (y < _displayclipy2); yy++) {
				uint16_t bo_save = bo;
				uint8_t bit_save = bit;
				uint8_t bits_save = bits;
				for (uint8_t yts = 0; (yts < textsize_y) && (y < _displayclipy2); yts++) {
					pfbPixel = pfbPixel_row;
					// need to repeat the stuff for each row...
					bo = bo_save;
					bit = bit_save;
					bits = bits_save;
					x = x_start;
					if (y >= _displayclipy1) {
						while (x < x_left_fill) {
							if ( (x >= _displayclipx1) && (x < _displayclipx2)) {
								if ((x >= _gfx_last_char_x_write) || (*pfbPixel != _gfx_last_char_textcolor))
									*pfbPixel = textbgcolor;
							}
							pfbPixel++;
							x++;

						}
				        for(xx=0; xx<w; xx++) {
				            if(!(bit++ & 7)) {
				                bits = bitmap[bo++];
				            }
				            for (uint8_t xts = 0; xts < textsize_x; xts++) {
								if ( (x >= _displayclipx1) && (x < _displayclipx2)) {
				            		if (bits & 0x80)
				            			*pfbPixel = textcolor;
				            		else if (x >= x_offset_cursor) {
										if ((x >= _gfx_last_char_x_write) || (*pfbPixel != _gfx_last_char_textcolor))
											*pfbPixel = textbgcolor;
				            		}
				            	}
								pfbPixel++;
				            	x++;	// remember our logical position...
				            }
				            bits <<= 1;
				        }
				        // Fill in any additional bg colors to right of our output
				        while (x++ < x_end) {
							if (x >= _displayclipx1) {
				        		*pfbPixel = textbgcolor;
				        	}
							pfbPixel++;
				        }
				    }
			        y++;	// remember which row we just output
					pfbPixel_row += _width;
			    }
		    }
		    // And output any more rows below us...
			while (y < y_end) {
				if (y >= _displayclipy1) {
					pfbPixel = pfbPixel_row;
					for (int16_t xx = x_start; xx < x_end; xx++) {
						if ((xx >= _displayclipx1) && (xx >= x_offset_cursor)) {
							if ((xx >= _gfx_last_char_x_write) || (*pfbPixel != _gfx_last_char_textcolor))
			        			*pfbPixel = textbgcolor;
			        	}
						pfbPixel++;
					}					
				}
				pfbPixel_row += _width;
				y++;
			}

		} else 
		#endif
		{
			// lets try to output text in one output rectangle
			//Serial.printf("    SPI (%d %d) (%d %d)\n", x_start, y_start, x_end, y_end);Serial.flush();
			// compute the actual region we will output given 
			beginSPITransaction();
		
			setAddr((x_start >= _displayclipx1) ? x_start : _displayclipx1, 
					(y_start >= _displayclipy1) ? y_start : _displayclipy1, 
					x_end  - 1,  y_end - 1); 
			writecommand(ST7735_RAMWR);
			//Serial.printf("SetAddr: %u %u %u %u\n", (x_start >= _displayclipx1) ? x_start : _displayclipx1, 
			//		(y_start >= _displayclipy1) ? y_start : _displayclipy1, 
			//		x_end  - 1,  y_end - 1); 
			// First lets fill in the top parts above the actual rectangle...
			//Serial.printf("    y_top_fill %d x_left_fill %d\n", y_top_fill, x_left_fill);
			while (y_top_fill--) {
				if ( (y >= _displayclipy1) && (y < _displayclipy2)) {
					for (int16_t xx = x_start; xx < x_end; xx++) {
						if (xx >= _displayclipx1) {
							writedata16(gfxFontLastCharPosFG(xx,y)? _gfx_last_char_textcolor : (xx < x_offset_cursor)? _gfx_last_char_textbgcolor : textbgcolor);
						}
					}					
				}
				y++;
			}
			//Serial.println("    After top fill"); Serial.flush();
			// Now lets output all of the pixels for each of the rows.. 
			for(yy=0; (yy<h) && (y < _displayclipy2); yy++) {
				uint16_t bo_save = bo;
				uint8_t bit_save = bit;
				uint8_t bits_save = bits;
				for (uint8_t yts = 0; (yts < textsize_y) && (y < _displayclipy2); yts++) {
					// need to repeat the stuff for each row...
					bo = bo_save;
					bit = bit_save;
					bits = bits_save;
					x = x_start;
					if (y >= _displayclipy1) {
						while (x < x_left_fill) {
							if ( (x >= _displayclipx1) && (x < _displayclipx2) ) {
								// Don't need to check if we are in previous char as in this case x_left_fill is set to 0...
								writedata16(gfxFontLastCharPosFG(x,y)? _gfx_last_char_textcolor :  textbgcolor);
							}
							x++;
						}
				        for(xx=0; xx<w; xx++) {
				            if(!(bit++ & 7)) {
				                bits = bitmap[bo++];
				            }
				            for (uint8_t xts = 0; xts < textsize_x; xts++) {
								if ( (x >= _displayclipx1) && (x < _displayclipx2)) {
				            		if (bits & 0x80)
										writedata16(textcolor);
									else 
										writedata16(gfxFontLastCharPosFG(x,y)? _gfx_last_char_textcolor : (x < x_offset_cursor)? _gfx_last_char_textbgcolor : textbgcolor);
								}
				            	x++;	// remember our logical position...
				            }
				            bits <<= 1;
				        }
				        // Fill in any additional bg colors to right of our output
				        while (x < x_end) {
							if (x >= _displayclipx1) {
								writedata16(gfxFontLastCharPosFG(x,y)? _gfx_last_char_textcolor : (x < x_offset_cursor)? _gfx_last_char_textbgcolor : textbgcolor);
				        	}
				        	x++;
				        }
			    	}
		        	y++;	// remember which row we just output
			    }
		    }
		    // And output any more rows below us...
		    //Serial.println("    Bottom fill"); Serial.flush();
			while (y < y_end) {
				if (y >= _displayclipy1) {
					for (int16_t xx = x_start; xx < x_end; xx++) {
						if (xx >= _displayclipx1 ) {
							writedata16(gfxFontLastCharPosFG(xx,y)? _gfx_last_char_textcolor : (xx < x_offset_cursor)? _gfx_last_char_textbgcolor : textbgcolor);
						}
					}
				}
				y++;
			}
			writecommand_last(ST7735_NOP);
			endSPITransaction();
		}
		// Save away info about this last char
		_gfx_c_last = c;
		_gfx_last_cursor_x = cursor_x + _originx; 
		_gfx_last_cursor_y = cursor_y + _originy;
		_gfx_last_char_x_write = x_end;
		_gfx_last_char_textcolor = textcolor;
		_gfx_last_char_textbgcolor = textbgcolor;
	}

    cursor_x += glyph->xAdvance * (int16_t)textsize_x;
}

// Some fonts overlap characters if we detect that the previous 
// character wrote out more width than they advanced in X direction
// we may want to know if the last character output a FG or BG at a position. 
	// Opaque font chracter overlap?
//	unsigned int _gfx_c_last;
//	int16_t   _gfx_last_cursor_x, _gfx_last_cursor_y;
//	int16_t	 _gfx_last_x_overlap = 0;
	
bool ST7735_t3::gfxFontLastCharPosFG(int16_t x, int16_t y) {
    GFXglyph *glyph  = gfxFont->glyph + (_gfx_c_last -  gfxFont->first);

    uint8_t   w     = glyph->width,
              h     = glyph->height;


    int16_t xo = glyph->xOffset; // sic
    int16_t yo = glyph->yOffset + gfxFont->yAdvance/2;
    if (x >= _gfx_last_char_x_write) return false; 	// we did not update here...
    if (y < (_gfx_last_cursor_y + (yo*textsize_y)))  return false;  // above
    if (y >= (_gfx_last_cursor_y + (yo+h)*textsize_y)) return false; // below


    // Lets compute which Row this y is in the bitmap
    int16_t y_bitmap = (y - ((_gfx_last_cursor_y + (yo*textsize_y))) + textsize_y - 1) / textsize_y;
    int16_t x_bitmap = (x - ((_gfx_last_cursor_x + (xo*textsize_x))) + textsize_x - 1) / textsize_x;
    uint16_t  pixel_bit_offset = y_bitmap * w + x_bitmap;

    return ((gfxFont->bitmap[glyph->bitmapOffset + (pixel_bit_offset >> 3)]) & (0x80 >> (pixel_bit_offset & 0x7)));
}



#ifdef ENABLE_ST77XX_FRAMEBUFFER
void ST7735_t3::dmaInterrupt(void) {
	if (_dmaActiveDisplay[0])  {
		_dmaActiveDisplay[0]->process_dma_interrupt();
	}
}
void ST7735_t3::dmaInterrupt1(void) {
	if (_dmaActiveDisplay[1])  {
		_dmaActiveDisplay[1]->process_dma_interrupt();
	}
}
void ST7735_t3::dmaInterrupt2(void) {
	if (_dmaActiveDisplay[2])  {
		_dmaActiveDisplay[2]->process_dma_interrupt();
	}
}

//=============================================================================
// Frame buffer support. 
//=============================================================================
#ifdef ENABLE_ST77XX_FRAMEBUFFER
#ifdef DEBUG_ASYNC_UPDATE
extern void dumpDMA_TCD(DMABaseClass *dmabc);
#endif

void ST7735_t3::process_dma_interrupt(void) {
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_2, HIGH);
#endif
	// Serial.println(" ST7735_t3::process_dma_interrupt");
#if defined(__MK66FX1M0__) 
	// T3.6
	_dmatx.clearInterrupt();

	#ifdef DEBUG_ASYNC_UPDATE
	static uint8_t debug_count = 10;
  if (_frame_callback_on_HalfDone && debug_count) {
  	Serial.printf("DI %x %x %x\n", _dmatx.TCD->SADDR, _dmasettings[_spi_num][0].TCD->SADDR, _dmasettings[_spi_num][1].TCD->SADDR);
  	debug_count--;
  }
  #endif
  if (_frame_callback_on_HalfDone &&
      (_dmatx.TCD->SADDR >= _dmasettings[_spi_num][1].TCD->SADDR)) {
    _dma_sub_frame_count = 1; // set as partial frame.
  } else {
		_dma_frame_count++;
	  _dma_sub_frame_count = 0; // set back to zero

		// See if we are in continuous mode or not..
		if ((_dma_state & ST77XX_DMA_CONT) == 0) {
			// We are in single refresh mode or the user has called cancel so
			// Lets try to release the CS pin
			while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
			writecommand_last(ST7735_NOP);
			endSPITransaction();
			_dma_state &= ~ST77XX_DMA_ACTIVE;
			_dmaActiveDisplay[_spi_num] = 0;	// We don't have a display active any more... 
		}
	}
  if (_frame_complete_callback)
    (*_frame_complete_callback)();

#elif defined(__IMXRT1062__)  // Teensy 4.x
  _dma_data[_spi_num]._dmatx.clearInterrupt();
  if (_frame_callback_on_HalfDone &&
      (_dma_data[_spi_num]._dmatx.TCD->SADDR >= _dma_data[_spi_num]._dmasettings[1].TCD->SADDR)) {
    _dma_sub_frame_count = 1; // set as partial frame.
    if (_frame_complete_callback)
      (*_frame_complete_callback)();
    // Serial.print("-");
  } else {

    _dma_frame_count++;
    _dma_sub_frame_count = 0;
    // See if we are in continuous mode or not..
    if ((_dma_state & ST77XX_DMA_CONT) == 0) {
      // We are in single refresh mode or the user has called cancel so
      // Lets try to release the CS pin
      // Serial.printf("Before FSR wait: %x %x\n", _pimxrt_spi->FSR,
      // _pimxrt_spi->SR);
      while (_pimxrt_spi->FSR & 0x1f)
        ; // wait until this one is complete

      // Serial.printf("Before SR busy wait: %x\n", _pimxrt_spi->SR);
      while (_pimxrt_spi->SR & LPSPI_SR_MBF)
        ; // wait until this one is complete

      _dma_data[_spi_num]._dmatx.clearComplete();
      // Serial.println("Restore FCR");
      _pimxrt_spi->FCR = LPSPI_FCR_TXWATER(
          15);              // _spi_fcr_save;	// restore the FSR status...
      _pimxrt_spi->DER = 0; // DMA no longer doing TX (or RX)

      _pimxrt_spi->CR =
          LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF; // actually clear both...
      _pimxrt_spi->SR = 0x3f00; // clear out all of the other status...

      maybeUpdateTCR(_tcr_dc_assert |
                     LPSPI_TCR_FRAMESZ(7)); // output Command with 8 bits
      // Serial.printf("Output NOP (SR %x CR %x FSR %x FCR %x %x TCR:%x)\n",
      // _pimxrt_spi->SR, _pimxrt_spi->CR, _pimxrt_spi->FSR,
      //	_pimxrt_spi->FCR, _spi_fcr_save, _pimxrt_spi->TCR);
      writecommand_last(ST7735_NOP);
      endSPITransaction();
      _dma_state &= ~ST77XX_DMA_ACTIVE;
      _dmaActiveDisplay[_spi_num] =
          0; // We don't have a display active any more...
    } else {
      // Lets try to flush out memory
      if (_frame_complete_callback)
        (*_frame_complete_callback)();
      else if ((uint32_t)_pfbtft >= 0x20200000u)
        arm_dcache_flush(_pfbtft, _count_pixels*2);
    }
  }
  asm("dsb");
#else
	//--------------------------------------------------------------------
	// T3.5...
	_dmarx.clearInterrupt();
	_dmatx.clearComplete();
	_dmarx.clearComplete();

	if (!_dma_count_remaining && !(_dma_state & ST77XX_DMA_CONT)) {
		// The DMA transfers are done.
		_dma_frame_count++;
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, HIGH);
#endif

		_pkinetisk_spi->RSER = 0;
		//_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);  // clear out the queue
		_pkinetisk_spi->SR = 0xFF0F0000;
		_pkinetisk_spi->CTAR0  &= ~(SPI_CTAR_FMSZ(8)); 	// Hack restore back to 8 bits

		writecommand_last(ST7735_NOP);
		endSPITransaction();
		_dma_state &= ~ST77XX_DMA_ACTIVE;
		_dmaActiveDisplay[_spi_num] = 0;	// We don't have a display active any more... 
    _dma_sub_frame_count = 0;
    if (_frame_complete_callback)
      (*_frame_complete_callback)();
#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_3, LOW);
#endif

	} else {
		uint16_t w;
		if (_dma_count_remaining) { // Still part of one frome. 
      bool half_done = _dma_count_remaining == (_count_pixels / 2);
			_dma_count_remaining -= _dma_write_size_words;
			w = *((uint16_t*)_dmatx.TCD->SADDR);
			_dmatx.TCD->SADDR = (volatile uint8_t*)(_dmatx.TCD->SADDR) + 2;
      if (_frame_complete_callback && _frame_callback_on_HalfDone &&
          half_done) {
        _dma_sub_frame_count = 1;
        (*_frame_complete_callback)();
      }
		} else {  // start a new frame
			_dma_frame_count++;
			_dmatx.sourceBuffer(&_pfbtft[1], (_dma_write_size_words-1)*2);
			_dmatx.TCD->SLAST = 0;	// Finish with it pointing to next location
			w = _pfbtft[0];
			_dma_count_remaining = _count_pixels - _dma_write_size_words;	// how much more to transfer? 
      _dma_sub_frame_count = 0;
      if (_frame_complete_callback)
        (*_frame_complete_callback)();
		}
#ifdef DEBUG_ASYNC_UPDATE
//		dumpDMA_TCD(&_dmatx);
//		dumpDMA_TCD(&_dmarx);
#endif
		_pkinetisk_spi->PUSHR = (w | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT);
		_dmarx.enable();
		_dmatx.enable();
	}

#endif	
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_2, LOW);
#endif
}

//=======================================================================
// Add optinal support for using frame buffer to speed up complex outputs
//=======================================================================
void ST7735_t3::setFrameBuffer(uint16_t *frame_buffer) 
{
	_pfbtft = frame_buffer;
	// we may not know the size of it, if called before init.
/*	if (_pfbtft != NULL) {
		memset(_pfbtft, 0, _screenWidth*_screenHeight*2);
	} */
}


void ST7735_t3::setFrameCompleteCB(void (*pcb)(), bool fCallAlsoHalfDone) {
  _frame_complete_callback = pcb;
  _frame_callback_on_HalfDone = pcb ? fCallAlsoHalfDone : false;

  noInterrupts();
  _dma_state &=
      ~ST77XX_DMA_INIT; // Lets setup  the call backs on next call out
  interrupts();
}

uint8_t ST7735_t3::useFrameBuffer(boolean b)		// use the frame buffer?  First call will allocate
{
	if (b) {
		// First see if we need to allocate buffer
		_count_pixels =  _width * _height;  // We may have called setFrameBuffer before we know with and height...
		if (_pfbtft == NULL) {
			// Hack to start frame buffer on 32 byte boundary
			// Note: If called before init maybe larger than we need
			_we_allocated_buffer = (uint16_t *)malloc(_count_pixels*2+32);
			if (_we_allocated_buffer == NULL)
				return 0;	// failed 
			_pfbtft = (uint16_t*) (((uintptr_t)_we_allocated_buffer + 32) & ~ ((uintptr_t) (31)));
			memset(_pfbtft, 0, _count_pixels*2);	
		}
		_use_fbtft = 1;
	} else 
		_use_fbtft = 0;

	return _use_fbtft;	
}

void ST7735_t3::freeFrameBuffer(void)						// explicit call to release the buffer
{
	if (_we_allocated_buffer) {
		free(_we_allocated_buffer);
		_pfbtft = NULL;
		_use_fbtft = 0;	// make sure the use is turned off
		_we_allocated_buffer = NULL;
	}
}
void ST7735_t3::updateScreen(void)					// call to say update the screen now.
{
	// Not sure if better here to check flag or check existence of buffer.
	// Will go by buffer as maybe can do interesting things?
	if (_use_fbtft) {
		beginSPITransaction();
		// Doing full window. 
		setAddr(0, 0, _width-1, _height-1);
		writecommand(ST7735_RAMWR);

		// BUGBUG doing as one shot.  Not sure if should or not or do like
		// main code and break up into transactions...
		uint16_t *pfbtft_end = &_pfbtft[(_count_pixels)-1];	// setup 
		uint16_t *pftbft = _pfbtft;

		// Quick write out the data;
		while (pftbft < pfbtft_end) {
			writedata16(*pftbft++);
		}
		writedata16_last(*pftbft);

		endSPITransaction();
	}
}			 

#ifdef DEBUG_ASYNC_UPDATE

void dumpDMA_TCD(DMABaseClass *dmabc)
{
	Serial.printf("%x %x:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

	Serial.printf("SA:%x SO:%d AT:%x NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n", (uint32_t)dmabc->TCD->SADDR,
		dmabc->TCD->SOFF, dmabc->TCD->ATTR, dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR, 
		dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA, dmabc->TCD->CSR, dmabc->TCD->BITER);
}
#endif

//==============================================
#ifdef ENABLE_ST77XX_FRAMEBUFFER
void	ST7735_t3::initDMASettings(void) 
{
	// Serial.printf("initDMASettings called %d\n", _dma_state);
  if (_dma_state & ST77XX_DMA_INIT) { // should test for init, but...
		return;	// we already init this. 
	}
#ifdef DEBUG_ASYNC_LEDS	
  pinMode(DEBUG_PIN_1, OUTPUT); digitalWrite(DEBUG_PIN_1, LOW);
  pinMode(DEBUG_PIN_2, OUTPUT); digitalWrite(DEBUG_PIN_2, LOW);
  pinMode(DEBUG_PIN_3, OUTPUT); digitalWrite(DEBUG_PIN_3, LOW);
#endif


	//Serial.println("InitDMASettings");
	uint8_t dmaTXevent = _spi_hardware->tx_dma_channel;
	_count_pixels = _width*_height;	// cache away the size of the display. 

//	Serial.printf("cbDisplay: %u COUNT_WORDS_WRITE:%d(%x) spi_num:%d\n", _count_pixels, COUNT_WORDS_WRITE, COUNT_WORDS_WRITE, _spi_num);
#if defined(__MK66FX1M0__) 
  uint8_t  cnt_dma_settings = 2;   // how many do we need for this display?
	uint32_t COUNT_WORDS_WRITE = (_count_pixels) / 2;
	// The 240x320 display requires us to expand to another DMA setting. 
	if (COUNT_WORDS_WRITE >= 32768) {
		COUNT_WORDS_WRITE = (_count_pixels) / 3;
		cnt_dma_settings = 3;
	}
	// T3.6
	//Serial.printf("CWW: %d %d %d\n", CBALLOC, SCREEN_DMA_NUM_SETTINGS, count_words_write);
	// Now lets setup DMA access to this memory... 
	_cnt_dma_settings = cnt_dma_settings;	// save away code that needs to update
	_dmasettings[_spi_num][0].sourceBuffer(&_pfbtft[1], (COUNT_WORDS_WRITE-1)*2);
	_dmasettings[_spi_num][0].destination(_pkinetisk_spi->PUSHR);

	// Hack to reset the destination to only output 2 bytes.
	_dmasettings[_spi_num][0].TCD->ATTR_DST = 1;
	_dmasettings[_spi_num][0].replaceSettingsOnCompletion(_dmasettings[_spi_num][1]);

	_dmasettings[_spi_num][1].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE], COUNT_WORDS_WRITE*2);
	_dmasettings[_spi_num][1].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[_spi_num][1].TCD->ATTR_DST = 1;
	_dmasettings[_spi_num][1].replaceSettingsOnCompletion(_dmasettings[_spi_num][2]);

	if (cnt_dma_settings == 3) {
		_dmasettings[_spi_num][2].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE*2], COUNT_WORDS_WRITE*2);
		_dmasettings[_spi_num][2].destination(_pkinetisk_spi->PUSHR);
		_dmasettings[_spi_num][2].TCD->ATTR_DST = 1;
		_dmasettings[_spi_num][2].replaceSettingsOnCompletion(_dmasettings[_spi_num][3]);		
		// 3 in chain so half done is half of 1...		
	  if (_frame_callback_on_HalfDone)
	    _dmasettings[_spi_num][1].interruptAtHalf();
	  else
	    _dmasettings[_spi_num][1].TCD->CSR &= ~DMA_TCD_CSR_INTHALF;
	} else {
	  if (_frame_callback_on_HalfDone)
	    _dmasettings[_spi_num][0].interruptAtCompletion();
	  else
	    _dmasettings[_spi_num][0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);
	}
	// Sort of hack - but wrap around to output the first word again. 
	_dmasettings[_spi_num][cnt_dma_settings].sourceBuffer(_pfbtft, 2);
	_dmasettings[_spi_num][cnt_dma_settings].destination(_pkinetisk_spi->PUSHR);
	_dmasettings[_spi_num][cnt_dma_settings].TCD->ATTR_DST = 1;
	_dmasettings[_spi_num][cnt_dma_settings].replaceSettingsOnCompletion(_dmasettings[_spi_num][0]);



	// Setup DMA main object
	//Serial.println("Setup _dmatx");
	_dmatx.begin(true);
	_dmatx.triggerAtHardwareEvent(dmaTXevent);
	_dmatx = _dmasettings[_spi_num][0];
	// probably could use const table of functio_ns...
	if (_spi_num == 0) _dmatx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dmatx.attachInterrupt(dmaInterrupt1);
	else _dmatx.attachInterrupt(dmaInterrupt2);

#elif defined(__IMXRT1062__)  // Teensy 4.x
	_cnt_dma_settings = 2;   // how many do we need for this display?

	uint32_t COUNT_WORDS_WRITE =  (height() * width()) / 2;
	// The 240x320 display requires us to expand to another DMA setting. 
	if (COUNT_WORDS_WRITE >= 32768) {
		COUNT_WORDS_WRITE = (height() * width()) / 3;
		_cnt_dma_settings = 3;
	}

  // First time we init...
  _dma_data[_spi_num]._dmasettings[0].sourceBuffer(_pfbtft, (COUNT_WORDS_WRITE)*2);
  _dma_data[_spi_num]._dmasettings[0].destination(_pimxrt_spi->TDR);
  _dma_data[_spi_num]._dmasettings[0].TCD->ATTR_DST = 1;
  _dma_data[_spi_num]._dmasettings[0].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[1]);

  _dma_data[_spi_num]._dmasettings[1].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE],
                               COUNT_WORDS_WRITE * 2);
  _dma_data[_spi_num]._dmasettings[1].destination(_pimxrt_spi->TDR);
  _dma_data[_spi_num]._dmasettings[1].TCD->ATTR_DST = 1;

	if (_cnt_dma_settings == 3) {
  	_dma_data[_spi_num]._dmasettings[1].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[2]);
    _dma_data[_spi_num]._dmasettings[2].sourceBuffer(&_pfbtft[COUNT_WORDS_WRITE * 2],
                                 COUNT_WORDS_WRITE * 2);
    _dma_data[_spi_num]._dmasettings[2].destination(_pimxrt_spi->TDR);
    _dma_data[_spi_num]._dmasettings[2].TCD->ATTR_DST = 1;
    _dma_data[_spi_num]._dmasettings[2].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[0]);
    _dma_data[_spi_num]._dmasettings[2].interruptAtCompletion();

		// 3 in chain so half done is half of 1...		
	  if (_frame_callback_on_HalfDone)
	    _dma_data[_spi_num]._dmasettings[1].interruptAtHalf();
	  else
	    _dma_data[_spi_num]._dmasettings[1].TCD->CSR &= ~DMA_TCD_CSR_INTHALF;
	} else {
    _dma_data[_spi_num]._dmasettings[1].replaceSettingsOnCompletion(_dma_data[_spi_num]._dmasettings[0]);
    _dma_data[_spi_num]._dmasettings[1].interruptAtCompletion();
	  if (_frame_callback_on_HalfDone)
	    _dma_data[_spi_num]._dmasettings[0].interruptAtCompletion();
	  else
	    _dma_data[_spi_num]._dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);
	}
	// Setup DMA main object
	//Serial.println("Setup _dmatx");
	// Serial.println("DMA initDMASettings - before dmatx");
	_dma_data[_spi_num]._dmatx.begin(true);
	_dma_data[_spi_num]._dmatx.triggerAtHardwareEvent(dmaTXevent);
	_dma_data[_spi_num]._dmatx = _dma_data[_spi_num]._dmasettings[0];
	// probably could use const table of functions...
	if (_spi_num == 0) _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt1);
	else _dma_data[_spi_num]._dmatx.attachInterrupt(dmaInterrupt2);
#else
	// T3.5
	// Lets setup the write size.  For SPI we can use up to 32767 so same size as we use on T3.6...
	// But SPI1 and SPI2 max of 511.  We will use 480 in that case as even divider...

	uint32_t COUNT_WORDS_WRITE = (_count_pixels) / 2;

	// The 240x320 display requires us to expand to another DMA interrupt...
	// Warning, hack, if interrupt at half, make it 4 interrupts...
	if (COUNT_WORDS_WRITE >= 32768) {
	  if (_frame_callback_on_HalfDone) COUNT_WORDS_WRITE = (_count_pixels) / 4;
	  else COUNT_WORDS_WRITE = (_count_pixels) / 3;
	}
	_dmarx.disable();
	_dmarx.source(_pkinetisk_spi->POPR);
	_dmarx.TCD->ATTR_SRC = 1;
	_dmarx.destination(_dma_dummy_rx);
	_dmarx.disableOnCompletion();
	_dmarx.triggerAtHardwareEvent(_spi_hardware->rx_dma_channel);
	// probably could use const table of functions...
	if (_spi_num == 0) _dmarx.attachInterrupt(dmaInterrupt);
	else if (_spi_num == 1) _dmarx.attachInterrupt(dmaInterrupt1);
	else _dmarx.attachInterrupt(dmaInterrupt2);

	_dmarx.interruptAtCompletion();

	// We may be using settings chain here so lets set it up. 
	// Now lets setup TX chain.  Note if trigger TX is not set
	// we need to have the RX do it for us.
	_dmatx.disable();
	_dmatx.destination(_pkinetisk_spi->PUSHR);
	_dmatx.TCD->ATTR_DST = 1;
	_dmatx.disableOnCompletion();
	// Current SPIN, has both RX/TX same for SPI1/2 so just know f
	if (_pspi == &SPI) {
		_dmatx.triggerAtHardwareEvent(dmaTXevent);
		_dma_write_size_words = COUNT_WORDS_WRITE;
	} else {
		_dma_write_size_words = 480;
	    _dmatx.triggerAtTransfersOf(_dmarx);
	}
	//Serial.printf("Init DMA Settings: TX:%d size:%d\n", dmaTXevent, _dma_write_size_words);

#endif
	_dma_state = ST77XX_DMA_INIT;  // Should be first thing set!
	// Serial.println("DMA initDMASettings - end");

}
#endif

void ST7735_t3::dumpDMASettings() {
#ifdef DEBUG_ASYNC_UPDATE
#if defined(__MK66FX1M0__) 
	// T3.6
	Serial.printf("DMA dump TCDs %d\n", _dmatx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmasettings[_spi_num][0]);
	dumpDMA_TCD(&_dmasettings[_spi_num][1]);
	dumpDMA_TCD(&_dmasettings[_spi_num][2]);
	dumpDMA_TCD(&_dmasettings[_spi_num][3]);
#elif defined(__IMXRT1062__)  // Teensy 4.x
	// Serial.printf("DMA dump TCDs %d\n", _dmatx.channel);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmatx);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmasettings[0]);
	dumpDMA_TCD(&_dma_data[_spi_num]._dmasettings[1]);
#else
	Serial.printf("DMA dump TX:%d RX:%d\n", _dmatx.channel, _dmarx.channel);
	dumpDMA_TCD(&_dmatx);
	dumpDMA_TCD(&_dmarx);
#endif	
#endif

}

bool ST7735_t3::updateScreenAsync(bool update_cont)					// call to say update the screen now.
{
	// Not sure if better here to check flag or check existence of buffer.
	// Will go by buffer as maybe can do interesting things?
	// BUGBUG:: only handles full screen so bail on the rest of it...
	// Also bail if we are working with a hardware SPI port. 
	#ifdef ENABLE_ST77XX_FRAMEBUFFER
	if (!_use_fbtft || !_pspi) return false;


	#if defined(__MK64FX512__) || defined(__MK20DX256__)  // If T3.5 only allow on SPI...
	// The T3.5 DMA to SPI has issues with preserving stuff like we want 16 bit mode
	// and we want CS to stay on... So hack it.  We will turn off using CS for the CS
	//	pin.
	if (!cspin && (_cs != 0xff)) {
		//Serial.println("***T3.5 CS Pin hack");
		pcs_data = 0;
		pcs_command = pcs_data | _pspi->setCS(_rs);
		pinMode(_cs, OUTPUT);
		cspin    = portOutputRegister(digitalPinToPort(_cs));
		*cspin = 1;
	}
	#endif

#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_1, HIGH);
#endif
	// Init DMA settings. 
	initDMASettings();

	// Don't start one if already active.
	if (_dma_state & ST77XX_DMA_ACTIVE) {
	#ifdef DEBUG_ASYNC_LEDS
		digitalWriteFast(DEBUG_PIN_1, LOW);
	#endif
		return false;
	}

#if defined(__MK66FX1M0__) 
	//==========================================
	// T3.6
	//==========================================
	if (update_cont) {
		// Try to link in #3 into the chain (_cnt_dma_settings)
		_dmasettings[_spi_num][_cnt_dma_settings-1].replaceSettingsOnCompletion(_dmasettings[_spi_num][_cnt_dma_settings]);
		_dmasettings[_spi_num][_cnt_dma_settings-1].TCD->CSR &= ~(DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ);  // Don't interrupt on this one... 
		_dmasettings[_spi_num][_cnt_dma_settings].interruptAtCompletion();
		_dmasettings[_spi_num][_cnt_dma_settings].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);  // Don't disable on this one  
		_dma_state |= ST77XX_DMA_CONT;
	} else {
		// In this case we will only run through once...
		_dmasettings[_spi_num][_cnt_dma_settings-1].replaceSettingsOnCompletion(_dmasettings[_spi_num][0]);
		_dmasettings[_spi_num][_cnt_dma_settings-1].interruptAtCompletion();
		_dmasettings[_spi_num][_cnt_dma_settings-1].disableOnCompletion();
		_dma_state &= ~ST77XX_DMA_CONT;
	}


#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif
	beginSPITransaction();

	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand(ST7735_RAMWR);

	// Write the first Word out before enter DMA as to setup the proper CS/DC/Continue flaugs
	writedata16(*_pfbtft);
	// now lets start up the DMA
//	volatile uint16_t  biter = _dmatx.TCD->BITER;
	//DMA_CDNE_CDNE(_dmatx.channel);
//	_dmatx = _dmasettings[0];
//	_dmatx.TCD->BITER = biter;
	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay[_spi_num] = this;
	_dma_state |= ST77XX_DMA_ACTIVE;
	_pkinetisk_spi->RSER |= SPI_RSER_TFFF_DIRS |	 SPI_RSER_TFFF_RE;	 // Set DMA Interrupt Request Select and Enable register
	_pkinetisk_spi->MCR &= ~SPI_MCR_HALT;  //Start transfers.
	_dmatx.enable();
	//==========================================
	// T4
	//==========================================
#elif defined(__IMXRT1062__)  // Teensy 4.x
  /////////////////////////////
  // BUGBUG try first not worry about continueous or not.
  // Start off remove disable on completion from both...
  // it will be the ISR that disables it...
  if ((uint32_t)_pfbtft >= 0x20200000u)
    arm_dcache_flush(_pfbtft, _count_pixels*2);

  _dma_data[_spi_num]._dmasettings[_cnt_dma_settings-1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ);
  beginSPITransaction();
// Doing full window.

  setAddr(0, 0, _width - 1, _height - 1);
  writecommand_last(ST7735_RAMWR);

  // Update TCR to 16 bit mode. and output the first entry.
  _spi_fcr_save = _pimxrt_spi->FCR; // remember the FCR
  _pimxrt_spi->FCR = 0;             // clear water marks...
  maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) |
                 LPSPI_TCR_RXMSK /*| LPSPI_TCR_CONT*/);
  _pimxrt_spi->DER = LPSPI_DER_TDDE;
  _pimxrt_spi->SR = 0x3f00; // clear out all of the other status...

  _dma_data[_spi_num]._dmatx.triggerAtHardwareEvent(_spi_hardware->tx_dma_channel);

  _dma_data[_spi_num]._dmatx =_dma_data[_spi_num]. _dmasettings[0];

  _dma_data[_spi_num]._dmatx.begin(false);
  _dma_data[_spi_num]._dmatx.enable();

  _dma_frame_count = 0; // Set frame count back to zero.
  _dmaActiveDisplay[_spi_num] = this;
  if (update_cont) {
    _dma_state |= ST77XX_DMA_CONT;
  } else {
    _dma_data[_spi_num]._dmasettings[_cnt_dma_settings-1].disableOnCompletion();
    _dma_state &= ~ST77XX_DMA_CONT;
  }

  _dma_state |= ST77XX_DMA_ACTIVE;
#ifdef DEBUG_ASYNC_UPDATE
  dumpDMASettings();
#endif
#else
	//==========================================
	// T3.5
	//==========================================

	// lets setup the initial pointers. 
	_dmatx.sourceBuffer(&_pfbtft[1], (_dma_write_size_words-1)*2);
	_dmatx.TCD->SLAST = 0;	// Finish with it pointing to next location
	_dmarx.transferCount(_dma_write_size_words);
	_dma_count_remaining = _count_pixels - _dma_write_size_words;	// how much more to transfer? 
	Serial.printf("updateScreenAsync:: - Pixels:%u Write Size:%u Remaining:%u DMA/Update: %u\n", _count_pixels, _dma_write_size_words, _dma_count_remaining, _count_pixels/_dma_write_size_words);

#ifdef DEBUG_ASYNC_UPDATE
	dumpDMASettings();
#endif

	beginSPITransaction();
	// Doing full window. 
	setAddr(0, 0, _width-1, _height-1);
	writecommand(ST7735_RAMWR);

	// Write the first Word out before enter DMA as to setup the proper CS/DC/Continue flaugs
	// On T3.5 DMA only appears to work with CTAR 0 so hack it up...
	_pkinetisk_spi->CTAR0 |= SPI_CTAR_FMSZ(8); 	// Hack convert from 8 bit to 16 bit...

	_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);

	_pkinetisk_spi->SR = 0xFF0F0000;

	// Lets try to output the first byte to make sure that we are in 16 bit mode...
	_pkinetisk_spi->PUSHR = *_pfbtft | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;	

	if (_pspi == &SPI) {
		// SPI - has both TX and RX so use it
		_pkinetisk_spi->RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS | SPI_RSER_TFFF_RE | SPI_RSER_TFFF_DIRS;

	    _dmarx.enable();
	    _dmatx.enable();
	} else {
		_pkinetisk_spi->RSER =  SPI_RSER_RFDF_RE | SPI_RSER_RFDF_DIRS ;
	    _dmatx.triggerAtTransfersOf(_dmarx);
	    _dmatx.enable();
	    _dmarx.enable();
	}

	_dma_frame_count = 0;  // Set frame count back to zero. 
	_dmaActiveDisplay[_spi_num] = this;
	if (update_cont) {
		_dma_state |= ST77XX_DMA_CONT;
	} else {
		_dma_state &= ~ST77XX_DMA_CONT;

	}

	_dma_state |= ST77XX_DMA_ACTIVE;
#endif	
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_1, LOW);
#endif
	return true;
    #else
    return false;     // no frame buffer so will never start... 
	#endif

}			 

void ST7735_t3::endUpdateAsync() {
	// make sure it is on
	#ifdef ENABLE_ST77XX_FRAMEBUFFER
	if (_dma_state & ST77XX_DMA_CONT) {
		_dma_state &= ~ST77XX_DMA_CONT; // Turn off the continueous mode
#if defined(__MK66FX1M0__)
		_dmasettings[_spi_num][_cnt_dma_settings].disableOnCompletion();
#endif
#if defined(__IMXRT1062__)
    _dma_data[_spi_num]._dmasettings[_cnt_dma_settings-1].disableOnCompletion();
#endif
	}
	#endif
}
	
void ST7735_t3::waitUpdateAsyncComplete(void) 
{
	#ifdef ENABLE_ST77XX_FRAMEBUFFER
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_3, HIGH);
#endif

	while ((_dma_state & ST77XX_DMA_ACTIVE)) {
		// asm volatile("wfi");
	};
#ifdef DEBUG_ASYNC_LEDS
	digitalWriteFast(DEBUG_PIN_3, LOW);
#endif
	#endif	
}

#endif

#endif

