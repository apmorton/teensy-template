/* Generated automatically from machmode.def and config/arm/arm-modes.def
   by genmodes.  */

#ifndef GCC_INSN_MODES_H
#define GCC_INSN_MODES_H

enum machine_mode
{
  VOIDmode,                /* machmode.def:172 */
  BLKmode,                 /* machmode.def:176 */
  CCmode,                  /* machmode.def:204 */
  CC_NOOVmode,             /* config/arm/arm-modes.def:46 */
  CC_Zmode,                /* config/arm/arm-modes.def:47 */
  CC_CZmode,               /* config/arm/arm-modes.def:48 */
  CC_NCVmode,              /* config/arm/arm-modes.def:49 */
  CC_SWPmode,              /* config/arm/arm-modes.def:50 */
  CCFPmode,                /* config/arm/arm-modes.def:51 */
  CCFPEmode,               /* config/arm/arm-modes.def:52 */
  CC_DNEmode,              /* config/arm/arm-modes.def:53 */
  CC_DEQmode,              /* config/arm/arm-modes.def:54 */
  CC_DLEmode,              /* config/arm/arm-modes.def:55 */
  CC_DLTmode,              /* config/arm/arm-modes.def:56 */
  CC_DGEmode,              /* config/arm/arm-modes.def:57 */
  CC_DGTmode,              /* config/arm/arm-modes.def:58 */
  CC_DLEUmode,             /* config/arm/arm-modes.def:59 */
  CC_DLTUmode,             /* config/arm/arm-modes.def:60 */
  CC_DGEUmode,             /* config/arm/arm-modes.def:61 */
  CC_DGTUmode,             /* config/arm/arm-modes.def:62 */
  CC_Cmode,                /* config/arm/arm-modes.def:63 */
  CC_Nmode,                /* config/arm/arm-modes.def:64 */
  BImode,                  /* machmode.def:179 */
  QImode,                  /* machmode.def:184 */
  HImode,                  /* machmode.def:185 */
  SImode,                  /* machmode.def:186 */
  DImode,                  /* machmode.def:187 */
  TImode,                  /* machmode.def:188 */
  EImode,                  /* config/arm/arm-modes.def:81 */
  OImode,                  /* config/arm/arm-modes.def:82 */
  CImode,                  /* config/arm/arm-modes.def:83 */
  XImode,                  /* config/arm/arm-modes.def:84 */
  QQmode,                  /* machmode.def:207 */
  HQmode,                  /* machmode.def:208 */
  SQmode,                  /* machmode.def:209 */
  DQmode,                  /* machmode.def:210 */
  TQmode,                  /* machmode.def:211 */
  UQQmode,                 /* machmode.def:213 */
  UHQmode,                 /* machmode.def:214 */
  USQmode,                 /* machmode.def:215 */
  UDQmode,                 /* machmode.def:216 */
  UTQmode,                 /* machmode.def:217 */
  HAmode,                  /* machmode.def:219 */
  SAmode,                  /* machmode.def:220 */
  DAmode,                  /* machmode.def:221 */
  TAmode,                  /* machmode.def:222 */
  UHAmode,                 /* machmode.def:224 */
  USAmode,                 /* machmode.def:225 */
  UDAmode,                 /* machmode.def:226 */
  UTAmode,                 /* machmode.def:227 */
  HFmode,                  /* config/arm/arm-modes.def:29 */
  SFmode,                  /* machmode.def:199 */
  DFmode,                  /* machmode.def:200 */
  XFmode,                  /* config/arm/arm-modes.def:26 */
  SDmode,                  /* machmode.def:239 */
  DDmode,                  /* machmode.def:240 */
  TDmode,                  /* machmode.def:241 */
  CQImode,                 /* machmode.def:235 */
  CHImode,                 /* machmode.def:235 */
  CSImode,                 /* machmode.def:235 */
  CDImode,                 /* machmode.def:235 */
  CTImode,                 /* machmode.def:235 */
  CEImode,                 /* machmode.def:235 */
  COImode,                 /* machmode.def:235 */
  CCImode,                 /* machmode.def:235 */
  CXImode,                 /* machmode.def:235 */
  HCmode,                  /* machmode.def:236 */
  SCmode,                  /* machmode.def:236 */
  DCmode,                  /* machmode.def:236 */
  XCmode,                  /* machmode.def:236 */
  V4QImode,                /* config/arm/arm-modes.def:67 */
  V2HImode,                /* config/arm/arm-modes.def:67 */
  V8QImode,                /* config/arm/arm-modes.def:68 */
  V4HImode,                /* config/arm/arm-modes.def:68 */
  V2SImode,                /* config/arm/arm-modes.def:68 */
  V16QImode,               /* config/arm/arm-modes.def:69 */
  V8HImode,                /* config/arm/arm-modes.def:69 */
  V4SImode,                /* config/arm/arm-modes.def:69 */
  V2DImode,                /* config/arm/arm-modes.def:69 */
  V4QQmode,                /* config/arm/arm-modes.def:74 */
  V2HQmode,                /* config/arm/arm-modes.def:74 */
  V4UQQmode,               /* config/arm/arm-modes.def:75 */
  V2UHQmode,               /* config/arm/arm-modes.def:75 */
  V2HAmode,                /* config/arm/arm-modes.def:76 */
  V2UHAmode,               /* config/arm/arm-modes.def:77 */
  V4HFmode,                /* config/arm/arm-modes.def:70 */
  V2SFmode,                /* config/arm/arm-modes.def:70 */
  V8HFmode,                /* config/arm/arm-modes.def:71 */
  V4SFmode,                /* config/arm/arm-modes.def:71 */
  V2DFmode,                /* config/arm/arm-modes.def:71 */
  MAX_MACHINE_MODE,

  MIN_MODE_RANDOM = VOIDmode,
  MAX_MODE_RANDOM = BLKmode,

  MIN_MODE_CC = CCmode,
  MAX_MODE_CC = CC_Nmode,

  MIN_MODE_INT = QImode,
  MAX_MODE_INT = XImode,

  MIN_MODE_PARTIAL_INT = VOIDmode,
  MAX_MODE_PARTIAL_INT = VOIDmode,

  MIN_MODE_FRACT = QQmode,
  MAX_MODE_FRACT = TQmode,

  MIN_MODE_UFRACT = UQQmode,
  MAX_MODE_UFRACT = UTQmode,

  MIN_MODE_ACCUM = HAmode,
  MAX_MODE_ACCUM = TAmode,

  MIN_MODE_UACCUM = UHAmode,
  MAX_MODE_UACCUM = UTAmode,

  MIN_MODE_FLOAT = HFmode,
  MAX_MODE_FLOAT = XFmode,

  MIN_MODE_DECIMAL_FLOAT = SDmode,
  MAX_MODE_DECIMAL_FLOAT = TDmode,

  MIN_MODE_COMPLEX_INT = CQImode,
  MAX_MODE_COMPLEX_INT = CXImode,

  MIN_MODE_COMPLEX_FLOAT = HCmode,
  MAX_MODE_COMPLEX_FLOAT = XCmode,

  MIN_MODE_VECTOR_INT = V4QImode,
  MAX_MODE_VECTOR_INT = V2DImode,

  MIN_MODE_VECTOR_FRACT = V4QQmode,
  MAX_MODE_VECTOR_FRACT = V2HQmode,

  MIN_MODE_VECTOR_UFRACT = V4UQQmode,
  MAX_MODE_VECTOR_UFRACT = V2UHQmode,

  MIN_MODE_VECTOR_ACCUM = V2HAmode,
  MAX_MODE_VECTOR_ACCUM = V2HAmode,

  MIN_MODE_VECTOR_UACCUM = V2UHAmode,
  MAX_MODE_VECTOR_UACCUM = V2UHAmode,

  MIN_MODE_VECTOR_FLOAT = V4HFmode,
  MAX_MODE_VECTOR_FLOAT = V2DFmode,

  NUM_MACHINE_MODES = MAX_MACHINE_MODE
};

#define CONST_MODE_SIZE const
#define CONST_MODE_BASE_ALIGN const
#define CONST_MODE_IBIT const
#define CONST_MODE_FBIT const

#endif /* insn-modes.h */
