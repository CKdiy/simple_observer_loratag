#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.c"











 


 

#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/std.h"











 




#include <stdarg.h>
#line 1 "C:\\IAR\\arm\\inc\\c\\stddef.h"
 
 




  #pragma system_include


#line 1 "C:\\IAR\\arm\\inc\\c\\ycheck.h"
 
 

 


  #pragma system_include















 





#line 38 "C:\\IAR\\arm\\inc\\c\\ycheck.h"
  
 
#line 11 "C:\\IAR\\arm\\inc\\c\\stddef.h"
#line 1 "C:\\IAR\\arm\\inc\\c\\yvals.h"
 
 





  #pragma system_include


 









  


 


 




 
#pragma rtmodel = "__dlib_version", "6"

 





 
#line 1 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"



























 





  #pragma system_include


 
#line 1 "C:/IAR/arm/inc/c/DLib_Config_Normal.h"
 
 





  #pragma system_include


 

#line 40 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"
   
#line 47 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"

 
#line 1 "C:\\IAR\\arm\\inc\\c\\DLib_Product.h"
 





   #pragma system_include







 




 



 



 



 


 









 


 


 






 




 




 


 


 


 
#line 106 "C:\\IAR\\arm\\inc\\c\\DLib_Product.h"









#line 51 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"











 



















 














 


























 








 






 

#line 153 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"









 

#line 172 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"













 
















 








 
#line 223 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"













 
















 





















 














 








 
#line 311 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"








 
#line 331 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"






 








 















 








 
















 




#line 400 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"





 

#line 414 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"


   
#line 424 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"

#line 432 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"

  







 









 









 







 







 
















 


   
#line 504 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"




 










 

#line 528 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"






 










 













 

#line 568 "C:\\IAR\\arm\\inc\\c\\DLib_Defaults.h"



 








#line 43 "C:\\IAR\\arm\\inc\\c\\yvals.h"



















 

#line 83 "C:\\IAR\\arm\\inc\\c\\yvals.h"

 







 
#line 102 "C:\\IAR\\arm\\inc\\c\\yvals.h"














 


   
#line 128 "C:\\IAR\\arm\\inc\\c\\yvals.h"





 
#line 141 "C:\\IAR\\arm\\inc\\c\\yvals.h"










 
#line 196 "C:\\IAR\\arm\\inc\\c\\yvals.h"

#line 204 "C:\\IAR\\arm\\inc\\c\\yvals.h"

#line 211 "C:\\IAR\\arm\\inc\\c\\yvals.h"

 


 




 
#line 232 "C:\\IAR\\arm\\inc\\c\\yvals.h"
  typedef unsigned long int _Wchart;

    typedef unsigned long int _Wintt;
#line 255 "C:\\IAR\\arm\\inc\\c\\yvals.h"

 


 
typedef unsigned int     _Sizet;

 
typedef struct _Mbstatet
{  

    unsigned int _Wchar;   
    unsigned int _State;   
#line 275 "C:\\IAR\\arm\\inc\\c\\yvals.h"

#line 299 "C:\\IAR\\arm\\inc\\c\\yvals.h"
} _Mbstatet;






 


 
#line 321 "C:\\IAR\\arm\\inc\\c\\yvals.h"
  typedef struct __va_list __Va_list;














 
typedef struct
{       

    long long _Off;     



  _Mbstatet _Wstate;
} _Fpost;





 


 

  
    
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Locksyslock_Malloc(void);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Locksyslock_Stream(void);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Locksyslock_Debug(void);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Locksyslock_StaticGuard(void);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Unlocksyslock_Malloc(void);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Unlocksyslock_Stream(void);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Unlocksyslock_Debug(void);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Unlocksyslock_StaticGuard(void);

#line 373 "C:\\IAR\\arm\\inc\\c\\yvals.h"

  typedef void *__iar_Rmtx;

  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Initdynamiclock(__iar_Rmtx *);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Dstdynamiclock(__iar_Rmtx *);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Lockdynamiclock(__iar_Rmtx *);
  _Pragma("object_attribute = __weak") __intrinsic __nounwind void __iar_Unlockdynamiclock(__iar_Rmtx *);

  
#line 406 "C:\\IAR\\arm\\inc\\c\\yvals.h"

#line 446 "C:\\IAR\\arm\\inc\\c\\yvals.h"






 
#line 12 "C:\\IAR\\arm\\inc\\c\\stddef.h"
#line 1 "C:\\IAR\\arm\\inc\\c\\ysizet.h"
 
 





  #pragma system_include


#line 1 "C:\\IAR\\arm\\inc\\c\\ycheck.h"
 
 

 


  #pragma system_include















 





#line 38 "C:\\IAR\\arm\\inc\\c\\ycheck.h"
  
 
#line 12 "C:\\IAR\\arm\\inc\\c\\ysizet.h"


 



  typedef _Sizet size_t;




typedef unsigned int __data_size_t;




#line 13 "C:\\IAR\\arm\\inc\\c\\stddef.h"

 








 



  typedef   signed int ptrdiff_t;




  typedef   _Wchart wchar_t;









    typedef union
    {
      long long _ll;
      long double _ld;
      void *_vp;
    } _Max_align_t;
    typedef _Max_align_t max_align_t;



#line 58 "C:\\IAR\\arm\\inc\\c\\stddef.h"






 
#line 19 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/std.h"

 




 



typedef char            xdc_Char;
typedef unsigned char   xdc_UChar;
typedef short           xdc_Short;
typedef unsigned short  xdc_UShort;
typedef int             xdc_Int;
typedef unsigned int    xdc_UInt;
typedef long            xdc_Long;
typedef unsigned long   xdc_ULong;
typedef float           xdc_Float;
typedef double          xdc_Double;
typedef long double     xdc_LDouble;
typedef size_t          xdc_SizeT;
typedef va_list         xdc_VaList;

 

typedef unsigned short  xdc_Bool;        
typedef void            *xdc_Ptr;        
typedef const void      *xdc_CPtr;       
typedef char            *xdc_String;     
typedef const char      *xdc_CString;    









 
#line 70 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/std.h"
  typedef int             (*xdc_Fxn)();    




 
#line 1 "C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/iar/targets/arm/std.h"










 



 




 




 
#line 1 "C:\\ti\\tirtos_cc13xx_cc26xx_2_21_01_08\\products\\bios_6_46_01_38\\packages\\iar\\targets\\arm\\M3.h"







 







 








 






 






 






 






 






 








 
#line 86 "C:\\ti\\tirtos_cc13xx_cc26xx_2_21_01_08\\products\\bios_6_46_01_38\\packages\\iar\\targets\\arm\\M3.h"






 
#line 109 "C:\\ti\\tirtos_cc13xx_cc26xx_2_21_01_08\\products\\bios_6_46_01_38\\packages\\iar\\targets\\arm\\M3.h"






 


#line 28 "C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/iar/targets/arm/std.h"


#line 1 "C:\\IAR\\arm\\inc\\c\\stdint.h"
 
 




  #pragma system_include


#line 1 "C:\\IAR\\arm\\inc\\c\\ycheck.h"
 
 

 


  #pragma system_include















 





#line 38 "C:\\IAR\\arm\\inc\\c\\ycheck.h"
  
 
#line 11 "C:\\IAR\\arm\\inc\\c\\stdint.h"






 

  typedef signed char          int8_t;
  typedef unsigned char        uint8_t;



  typedef signed short int         int16_t;
  typedef unsigned short int       uint16_t;



  typedef signed int         int32_t;
  typedef unsigned int       uint32_t;



  typedef signed long long int         int64_t;
  typedef unsigned long long int       uint64_t;



 
typedef signed char      int_least8_t;
typedef unsigned char    uint_least8_t;

typedef signed short int     int_least16_t;
typedef unsigned short int   uint_least16_t;

typedef signed int     int_least32_t;
typedef unsigned int   uint_least32_t;

 

  typedef signed long long int   int_least64_t;


  typedef unsigned long long int uint_least64_t;




 
typedef signed int       int_fast8_t;
typedef unsigned int     uint_fast8_t;

typedef signed int      int_fast16_t;
typedef unsigned int    uint_fast16_t;

typedef signed int      int_fast32_t;
typedef unsigned int    uint_fast32_t;


  typedef signed long long int    int_fast64_t;


  typedef unsigned long long int  uint_fast64_t;


 
typedef signed long long int          intmax_t;
typedef unsigned long long int        uintmax_t;


 
typedef signed int          intptr_t;
typedef unsigned int        uintptr_t;

 



typedef int __data_intptr_t; typedef unsigned int __data_uintptr_t;


 

























































































 



































 
#line 31 "C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/iar/targets/arm/std.h"

 





 
#line 45 "C:/ti/tirtos_cc13xx_cc26xx_2_21_01_08/products/bios_6_46_01_38/packages/iar/targets/arm/std.h"


 
typedef int_least8_t        xdc_Int8;
typedef uint_least8_t       xdc_UInt8;
typedef int_least16_t       xdc_Int16;
typedef uint_least16_t      xdc_UInt16;
typedef int_least32_t       xdc_Int32;
typedef uint_least32_t      xdc_UInt32;
typedef int_least64_t       xdc_Int64;
typedef uint_least64_t      xdc_UInt64;



 
typedef uint8_t       xdc_Bits8;
typedef uint16_t      xdc_Bits16;
typedef uint32_t      xdc_Bits32;
typedef uint64_t      xdc_Bits64;



 
typedef intptr_t        xdc_IArg;
typedef uintptr_t       xdc_UArg;



 








 

#line 81 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/std.h"
#line 113 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/std.h"










 




 


typedef long long               xdc_LLong;
typedef unsigned long long      xdc_ULLong;

#line 147 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/std.h"






 

static inline xdc_Ptr xdc_iargToPtr(xdc_IArg a) { return ((xdc_Ptr)a); }
static inline xdc_Ptr xdc_uargToPtr(xdc_UArg a) { return ((xdc_Ptr)a); }



static inline xdc_Fxn xdc_iargToFxn(xdc_IArg a) { return ((xdc_Fxn)a); }
static inline xdc_Fxn xdc_uargToFxn(xdc_UArg a) { return ((xdc_Fxn)a); }








 
typedef union {
    xdc_Float f;
    xdc_IArg  a;
} xdc_FloatData;

static inline xdc_IArg xdc_floatToArg(xdc_Float f)
{
     xdc_FloatData u;
     u.f = f;

     return (u.a);
}

static inline xdc_Float xdc_argToFloat(xdc_IArg a)
{
     xdc_FloatData u;
     u.a = a;

     return (u.f);
}


 




 



#line 208 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/std.h"



typedef xdc_Char        Char;
typedef xdc_UChar       UChar;
typedef xdc_Short       Short;
typedef xdc_UShort      UShort;
typedef xdc_Int         Int;
typedef xdc_UInt        UInt;
typedef xdc_Long        Long;
typedef xdc_ULong       ULong;
typedef xdc_LLong       LLong;
typedef xdc_ULLong      ULLong;
typedef xdc_Float       Float;
typedef xdc_Double      Double;
typedef xdc_LDouble     LDouble;
typedef xdc_SizeT       SizeT;
typedef xdc_VaList      VaList;

typedef xdc_IArg        IArg;
typedef xdc_UArg        UArg;
typedef xdc_Bool        Bool;
typedef xdc_Int8        Int8;
typedef xdc_Int16       Int16;
typedef xdc_Int32       Int32;
typedef xdc_Fxn         Fxn;
typedef xdc_Ptr         Ptr;

typedef xdc_String      String;

typedef xdc_CString     CString;

typedef xdc_UInt8       UInt8;
typedef xdc_UInt16      UInt16;
typedef xdc_UInt32      UInt32;

 



 




typedef xdc_UInt8       Uint8;
typedef xdc_UInt16      Uint16;
typedef xdc_UInt32      Uint32;
typedef xdc_UInt        Uns;





 

typedef xdc_Int64       Int64;
typedef xdc_UInt64      UInt64;





 

typedef xdc_Bits8       Bits8;



typedef xdc_Bits16      Bits16;



typedef xdc_Bits32      Bits32;



typedef xdc_Bits64      Bits64;




 






 
#line 304 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/std.h"







 

















  

















 







 

#line 18 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.c"
#line 1 "C:\\ti\\xdctools_3_32_00_06_core\\packages\\xdc\\runtime\\package/internal/SysCallback.xdc.h"





 








#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.h"





 

















 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/xdc.h"











 



 







 






 





























 
#line 69 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/xdc.h"




 















 

#line 55 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types__prologue.h"











 



 


 

#line 54 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/package.defs.h"





 







 

typedef struct xdc_runtime_IModule_Fxns__ xdc_runtime_IModule_Fxns__;
typedef const xdc_runtime_IModule_Fxns__* xdc_runtime_IModule_Module;



 




 

typedef struct xdc_runtime_Core_ObjDesc xdc_runtime_Core_ObjDesc;



 




 

typedef struct xdc_runtime_Diags_DictElem xdc_runtime_Diags_DictElem;



 

typedef struct xdc_runtime_Error_Data xdc_runtime_Error_Data;
typedef struct xdc_runtime_Error_Block xdc_runtime_Error_Block;
typedef struct xdc_runtime_Error_Module_State xdc_runtime_Error_Module_State;



 




 

typedef struct xdc_runtime_IGateProvider_Fxns__ xdc_runtime_IGateProvider_Fxns__;
typedef const xdc_runtime_IGateProvider_Fxns__* xdc_runtime_IGateProvider_Module;
typedef struct xdc_runtime_IGateProvider_Params xdc_runtime_IGateProvider_Params;
typedef struct xdc_runtime_IGateProvider___Object { xdc_runtime_IGateProvider_Fxns__* __fxns; xdc_Bits32 __label; } *xdc_runtime_IGateProvider_Handle;



 

typedef struct xdc_runtime_GateNull_Fxns__ xdc_runtime_GateNull_Fxns__;
typedef const xdc_runtime_GateNull_Fxns__* xdc_runtime_GateNull_Module;
typedef struct xdc_runtime_GateNull_Params xdc_runtime_GateNull_Params;
typedef struct xdc_runtime_GateNull_Object xdc_runtime_GateNull_Object;
typedef struct xdc_runtime_GateNull_Struct xdc_runtime_GateNull_Struct;
typedef xdc_runtime_GateNull_Object* xdc_runtime_GateNull_Handle;
typedef struct xdc_runtime_GateNull_Object__ xdc_runtime_GateNull_Instance_State;
typedef xdc_runtime_GateNull_Object* xdc_runtime_GateNull_Instance;



 

typedef struct xdc_runtime_Log_EventRec xdc_runtime_Log_EventRec;



 

typedef struct xdc_runtime_ILogger_Fxns__ xdc_runtime_ILogger_Fxns__;
typedef const xdc_runtime_ILogger_Fxns__* xdc_runtime_ILogger_Module;
typedef struct xdc_runtime_ILogger_Params xdc_runtime_ILogger_Params;
typedef struct xdc_runtime_ILogger___Object { xdc_runtime_ILogger_Fxns__* __fxns; xdc_Bits32 __label; } *xdc_runtime_ILogger_Handle;



 

typedef struct xdc_runtime_IFilterLogger_Fxns__ xdc_runtime_IFilterLogger_Fxns__;
typedef const xdc_runtime_IFilterLogger_Fxns__* xdc_runtime_IFilterLogger_Module;
typedef struct xdc_runtime_IFilterLogger_Params xdc_runtime_IFilterLogger_Params;
typedef struct xdc_runtime_IFilterLogger___Object { xdc_runtime_IFilterLogger_Fxns__* __fxns; xdc_Bits32 __label; } *xdc_runtime_IFilterLogger_Handle;



 

typedef struct xdc_runtime_LoggerBuf_Entry xdc_runtime_LoggerBuf_Entry;
typedef struct xdc_runtime_LoggerBuf_Module_State xdc_runtime_LoggerBuf_Module_State;
typedef struct xdc_runtime_LoggerBuf_Fxns__ xdc_runtime_LoggerBuf_Fxns__;
typedef const xdc_runtime_LoggerBuf_Fxns__* xdc_runtime_LoggerBuf_Module;
typedef struct xdc_runtime_LoggerBuf_Params xdc_runtime_LoggerBuf_Params;
typedef struct xdc_runtime_LoggerBuf_Object xdc_runtime_LoggerBuf_Object;
typedef struct xdc_runtime_LoggerBuf_Struct xdc_runtime_LoggerBuf_Struct;
typedef xdc_runtime_LoggerBuf_Object* xdc_runtime_LoggerBuf_Handle;
typedef struct xdc_runtime_LoggerBuf_Object__ xdc_runtime_LoggerBuf_Instance_State;
typedef xdc_runtime_LoggerBuf_Object* xdc_runtime_LoggerBuf_Instance;



 

typedef struct xdc_runtime_LoggerCallback_Fxns__ xdc_runtime_LoggerCallback_Fxns__;
typedef const xdc_runtime_LoggerCallback_Fxns__* xdc_runtime_LoggerCallback_Module;
typedef struct xdc_runtime_LoggerCallback_Params xdc_runtime_LoggerCallback_Params;
typedef struct xdc_runtime_LoggerCallback_Object xdc_runtime_LoggerCallback_Object;
typedef struct xdc_runtime_LoggerCallback_Struct xdc_runtime_LoggerCallback_Struct;
typedef xdc_runtime_LoggerCallback_Object* xdc_runtime_LoggerCallback_Handle;
typedef struct xdc_runtime_LoggerCallback_Object__ xdc_runtime_LoggerCallback_Instance_State;
typedef xdc_runtime_LoggerCallback_Object* xdc_runtime_LoggerCallback_Instance;



 

typedef struct xdc_runtime_LoggerSys_Fxns__ xdc_runtime_LoggerSys_Fxns__;
typedef const xdc_runtime_LoggerSys_Fxns__* xdc_runtime_LoggerSys_Module;
typedef struct xdc_runtime_LoggerSys_Params xdc_runtime_LoggerSys_Params;
typedef struct xdc_runtime_LoggerSys_Object xdc_runtime_LoggerSys_Object;
typedef struct xdc_runtime_LoggerSys_Struct xdc_runtime_LoggerSys_Struct;
typedef xdc_runtime_LoggerSys_Object* xdc_runtime_LoggerSys_Handle;
typedef struct xdc_runtime_LoggerSys_Object__ xdc_runtime_LoggerSys_Instance_State;
typedef xdc_runtime_LoggerSys_Object* xdc_runtime_LoggerSys_Instance;



 




 

typedef struct xdc_runtime_Memory_Stats xdc_runtime_Memory_Stats;
typedef struct xdc_runtime_Memory_Module_State xdc_runtime_Memory_Module_State;



 

typedef struct xdc_runtime_IHeap_Fxns__ xdc_runtime_IHeap_Fxns__;
typedef const xdc_runtime_IHeap_Fxns__* xdc_runtime_IHeap_Module;
typedef struct xdc_runtime_IHeap_Params xdc_runtime_IHeap_Params;
typedef struct xdc_runtime_IHeap___Object { xdc_runtime_IHeap_Fxns__* __fxns; xdc_Bits32 __label; } *xdc_runtime_IHeap_Handle;



 

typedef struct xdc_runtime_HeapMin_Fxns__ xdc_runtime_HeapMin_Fxns__;
typedef const xdc_runtime_HeapMin_Fxns__* xdc_runtime_HeapMin_Module;
typedef struct xdc_runtime_HeapMin_Params xdc_runtime_HeapMin_Params;
typedef struct xdc_runtime_HeapMin_Object xdc_runtime_HeapMin_Object;
typedef struct xdc_runtime_HeapMin_Struct xdc_runtime_HeapMin_Struct;
typedef xdc_runtime_HeapMin_Object* xdc_runtime_HeapMin_Handle;
typedef struct xdc_runtime_HeapMin_Object__ xdc_runtime_HeapMin_Instance_State;
typedef xdc_runtime_HeapMin_Object* xdc_runtime_HeapMin_Instance;



 

typedef struct xdc_runtime_HeapStd_Module_State xdc_runtime_HeapStd_Module_State;
typedef struct xdc_runtime_HeapStd_Fxns__ xdc_runtime_HeapStd_Fxns__;
typedef const xdc_runtime_HeapStd_Fxns__* xdc_runtime_HeapStd_Module;
typedef struct xdc_runtime_HeapStd_Params xdc_runtime_HeapStd_Params;
typedef struct xdc_runtime_HeapStd_Object xdc_runtime_HeapStd_Object;
typedef struct xdc_runtime_HeapStd_Struct xdc_runtime_HeapStd_Struct;
typedef xdc_runtime_HeapStd_Object* xdc_runtime_HeapStd_Handle;
typedef struct xdc_runtime_HeapStd_Object__ xdc_runtime_HeapStd_Instance_State;
typedef xdc_runtime_HeapStd_Object* xdc_runtime_HeapStd_Instance;



 

typedef struct xdc_runtime_Registry_Module_State xdc_runtime_Registry_Module_State;



 

typedef struct xdc_runtime_Rta_CommandPacket xdc_runtime_Rta_CommandPacket;
typedef struct xdc_runtime_Rta_ResponsePacket xdc_runtime_Rta_ResponsePacket;



 

typedef struct xdc_runtime_Startup_IdMap xdc_runtime_Startup_IdMap;
typedef struct xdc_runtime_Startup_Module_State xdc_runtime_Startup_Module_State;



 

typedef struct xdc_runtime_System_ParseData xdc_runtime_System_ParseData;
typedef struct xdc_runtime_System_Module_State xdc_runtime_System_Module_State;



 

typedef struct xdc_runtime_ISystemSupport_Fxns__ xdc_runtime_ISystemSupport_Fxns__;
typedef const xdc_runtime_ISystemSupport_Fxns__* xdc_runtime_ISystemSupport_Module;



 

typedef struct xdc_runtime_SysCallback_Fxns__ xdc_runtime_SysCallback_Fxns__;
typedef const xdc_runtime_SysCallback_Fxns__* xdc_runtime_SysCallback_Module;



 

typedef struct xdc_runtime_SysMin_Module_State xdc_runtime_SysMin_Module_State;
typedef struct xdc_runtime_SysMin_Fxns__ xdc_runtime_SysMin_Fxns__;
typedef const xdc_runtime_SysMin_Fxns__* xdc_runtime_SysMin_Module;



 

typedef struct xdc_runtime_SysStd_Fxns__ xdc_runtime_SysStd_Fxns__;
typedef const xdc_runtime_SysStd_Fxns__* xdc_runtime_SysStd_Module;



 

typedef struct xdc_runtime_Text_Node xdc_runtime_Text_Node;
typedef struct xdc_runtime_Text_MatchVisState xdc_runtime_Text_MatchVisState;
typedef struct xdc_runtime_Text_PrintVisState xdc_runtime_Text_PrintVisState;
typedef struct xdc_runtime_Text_Module_State xdc_runtime_Text_Module_State;



 

typedef struct xdc_runtime_ITimestampClient_Fxns__ xdc_runtime_ITimestampClient_Fxns__;
typedef const xdc_runtime_ITimestampClient_Fxns__* xdc_runtime_ITimestampClient_Module;



 

typedef struct xdc_runtime_Timestamp_Fxns__ xdc_runtime_Timestamp_Fxns__;
typedef const xdc_runtime_Timestamp_Fxns__* xdc_runtime_Timestamp_Module;



 

typedef struct xdc_runtime_ITimestampProvider_Fxns__ xdc_runtime_ITimestampProvider_Fxns__;
typedef const xdc_runtime_ITimestampProvider_Fxns__* xdc_runtime_ITimestampProvider_Module;



 

typedef struct xdc_runtime_TimestampNull_Fxns__ xdc_runtime_TimestampNull_Fxns__;
typedef const xdc_runtime_TimestampNull_Fxns__* xdc_runtime_TimestampNull_Module;



 

typedef struct xdc_runtime_TimestampStd_Fxns__ xdc_runtime_TimestampStd_Fxns__;
typedef const xdc_runtime_TimestampStd_Fxns__* xdc_runtime_TimestampStd_Module;



 

typedef struct xdc_runtime_Types_CordAddr__ xdc_runtime_Types_CordAddr__;
typedef struct xdc_runtime_Types_GateRef__ xdc_runtime_Types_GateRef__;
typedef struct xdc_runtime_Types_Label xdc_runtime_Types_Label;
typedef struct xdc_runtime_Types_Site xdc_runtime_Types_Site;
typedef struct xdc_runtime_Types_Timestamp64 xdc_runtime_Types_Timestamp64;
typedef struct xdc_runtime_Types_FreqHz xdc_runtime_Types_FreqHz;
typedef struct xdc_runtime_Types_RegDesc xdc_runtime_Types_RegDesc;
typedef struct xdc_runtime_Types_Vec xdc_runtime_Types_Vec;
typedef struct xdc_runtime_Types_Link xdc_runtime_Types_Link;
typedef struct xdc_runtime_Types_InstHdr xdc_runtime_Types_InstHdr;
typedef struct xdc_runtime_Types_PrmsHdr xdc_runtime_Types_PrmsHdr;
typedef struct xdc_runtime_Types_Base xdc_runtime_Types_Base;
typedef struct xdc_runtime_Types_SysFxns xdc_runtime_Types_SysFxns;
typedef struct xdc_runtime_Types_SysFxns2 xdc_runtime_Types_SysFxns2;



 

typedef struct xdc_runtime_IInstance_Fxns__ xdc_runtime_IInstance_Fxns__;
typedef const xdc_runtime_IInstance_Fxns__* xdc_runtime_IInstance_Module;
typedef struct xdc_runtime_IInstance_Params xdc_runtime_IInstance_Params;
typedef struct xdc_runtime_IInstance___Object { xdc_runtime_IInstance_Fxns__* __fxns; xdc_Bits32 __label; } *xdc_runtime_IInstance_Handle;



 

typedef struct xdc_runtime_LoggerBuf_TimestampProxy_Fxns__ xdc_runtime_LoggerBuf_TimestampProxy_Fxns__;
typedef const xdc_runtime_LoggerBuf_TimestampProxy_Fxns__* xdc_runtime_LoggerBuf_TimestampProxy_Module;



 

typedef struct xdc_runtime_LoggerBuf_Module_GateProxy_Fxns__ xdc_runtime_LoggerBuf_Module_GateProxy_Fxns__;
typedef const xdc_runtime_LoggerBuf_Module_GateProxy_Fxns__* xdc_runtime_LoggerBuf_Module_GateProxy_Module;
typedef struct xdc_runtime_LoggerBuf_Module_GateProxy_Params xdc_runtime_LoggerBuf_Module_GateProxy_Params;
typedef struct xdc_runtime_IGateProvider___Object *xdc_runtime_LoggerBuf_Module_GateProxy_Handle;



 

typedef struct xdc_runtime_LoggerSys_TimestampProxy_Fxns__ xdc_runtime_LoggerSys_TimestampProxy_Fxns__;
typedef const xdc_runtime_LoggerSys_TimestampProxy_Fxns__* xdc_runtime_LoggerSys_TimestampProxy_Module;



 

typedef struct xdc_runtime_Main_Module_GateProxy_Fxns__ xdc_runtime_Main_Module_GateProxy_Fxns__;
typedef const xdc_runtime_Main_Module_GateProxy_Fxns__* xdc_runtime_Main_Module_GateProxy_Module;
typedef struct xdc_runtime_Main_Module_GateProxy_Params xdc_runtime_Main_Module_GateProxy_Params;
typedef struct xdc_runtime_IGateProvider___Object *xdc_runtime_Main_Module_GateProxy_Handle;



 

typedef struct xdc_runtime_Memory_HeapProxy_Fxns__ xdc_runtime_Memory_HeapProxy_Fxns__;
typedef const xdc_runtime_Memory_HeapProxy_Fxns__* xdc_runtime_Memory_HeapProxy_Module;
typedef struct xdc_runtime_Memory_HeapProxy_Params xdc_runtime_Memory_HeapProxy_Params;
typedef struct xdc_runtime_IHeap___Object *xdc_runtime_Memory_HeapProxy_Handle;



 

typedef struct xdc_runtime_System_SupportProxy_Fxns__ xdc_runtime_System_SupportProxy_Fxns__;
typedef const xdc_runtime_System_SupportProxy_Fxns__* xdc_runtime_System_SupportProxy_Module;



 

typedef struct xdc_runtime_System_Module_GateProxy_Fxns__ xdc_runtime_System_Module_GateProxy_Fxns__;
typedef const xdc_runtime_System_Module_GateProxy_Fxns__* xdc_runtime_System_Module_GateProxy_Module;
typedef struct xdc_runtime_System_Module_GateProxy_Params xdc_runtime_System_Module_GateProxy_Params;
typedef struct xdc_runtime_IGateProvider___Object *xdc_runtime_System_Module_GateProxy_Handle;



 

typedef struct xdc_runtime_Timestamp_SupportProxy_Fxns__ xdc_runtime_Timestamp_SupportProxy_Fxns__;
typedef const xdc_runtime_Timestamp_SupportProxy_Fxns__* xdc_runtime_Timestamp_SupportProxy_Module;


#line 55 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

 
typedef xdc_Bits16 xdc_runtime_Types_ModuleId;

 
typedef xdc_Bits16 xdc_runtime_Types_DiagsMask;

 
typedef xdc_Bits32 xdc_runtime_Types_Event;

 
typedef xdc_runtime_Types_Event xdc_runtime_Types_EventId;

 
struct xdc_runtime_Types_CordAddr__;

 
typedef xdc_runtime_Types_CordAddr__ *xdc_runtime_Types_CordAddr;

 
struct xdc_runtime_Types_GateRef__;

 
typedef xdc_runtime_Types_GateRef__ *xdc_runtime_Types_GateRef;

 
typedef xdc_Bits16 xdc_runtime_Types_RopeId;

 
enum xdc_runtime_Types_CreatePolicy {
    xdc_runtime_Types_STATIC_POLICY,
    xdc_runtime_Types_CREATE_POLICY,
    xdc_runtime_Types_DELETE_POLICY
};
typedef enum xdc_runtime_Types_CreatePolicy xdc_runtime_Types_CreatePolicy;

 
enum xdc_runtime_Types_OutputPolicy {
    xdc_runtime_Types_COMMON_FILE,
    xdc_runtime_Types_SEPARATE_FILE,
    xdc_runtime_Types_NO_FILE
};
typedef enum xdc_runtime_Types_OutputPolicy xdc_runtime_Types_OutputPolicy;

 
struct xdc_runtime_Types_Label {
    xdc_Ptr handle;
    xdc_runtime_Types_ModuleId modId;
    xdc_String iname;
    xdc_Bool named;
};

 
struct xdc_runtime_Types_Site {
    xdc_runtime_Types_ModuleId mod;
    xdc_CString file;
    xdc_Int line;
};

 
struct xdc_runtime_Types_Timestamp64 {
    xdc_Bits32 hi;
    xdc_Bits32 lo;
};

 
struct xdc_runtime_Types_FreqHz {
    xdc_Bits32 hi;
    xdc_Bits32 lo;
};

 
struct xdc_runtime_Types_RegDesc {
    xdc_runtime_Types_RegDesc *next;
    xdc_CString modName;
    xdc_runtime_Types_ModuleId id;
    xdc_runtime_Types_DiagsMask mask;
};




 

 
typedef xdc_Bits32 xdc_runtime_Types_LogEvent;

 
typedef void (*xdc_runtime_Types_LoggerFxn0)(xdc_Ptr, xdc_runtime_Types_LogEvent, xdc_runtime_Types_ModuleId);

 
typedef void (*xdc_runtime_Types_LoggerFxn1)(xdc_Ptr, xdc_runtime_Types_LogEvent, xdc_runtime_Types_ModuleId, xdc_IArg);

 
typedef void (*xdc_runtime_Types_LoggerFxn2)(xdc_Ptr, xdc_runtime_Types_LogEvent, xdc_runtime_Types_ModuleId, xdc_IArg, xdc_IArg);

 
typedef void (*xdc_runtime_Types_LoggerFxn4)(xdc_Ptr, xdc_runtime_Types_LogEvent, xdc_runtime_Types_ModuleId, xdc_IArg, xdc_IArg, xdc_IArg, xdc_IArg);

 
typedef void (*xdc_runtime_Types_LoggerFxn8)(xdc_Ptr, xdc_runtime_Types_LogEvent, xdc_runtime_Types_ModuleId, xdc_IArg, xdc_IArg, xdc_IArg, xdc_IArg, xdc_IArg, xdc_IArg, xdc_IArg, xdc_IArg);

 
struct xdc_runtime_Types_Vec {
    xdc_Int len;
    xdc_Ptr arr;
};

 
struct xdc_runtime_Types_Link {
    xdc_runtime_Types_Link *next;
    xdc_runtime_Types_Link *prev;
};

 
struct xdc_runtime_Types_InstHdr {
    xdc_runtime_Types_Link link;
};

 
struct xdc_runtime_Types_PrmsHdr {
    xdc_SizeT size;
    xdc_Ptr self;
    xdc_Ptr modFxns;
    xdc_Ptr instPrms;
};

 
struct xdc_runtime_Types_Base {
    xdc_runtime_Types_Base *base;
};

 
struct xdc_runtime_Types_SysFxns {
    xdc_Ptr (*__create)(xdc_Ptr, xdc_SizeT, xdc_Ptr, const xdc_Ptr, xdc_SizeT, xdc_runtime_Error_Block*);
    void (*__delete)(xdc_Ptr);
    xdc_runtime_Types_Label *(*__label)(xdc_Ptr, xdc_runtime_Types_Label*);
    xdc_runtime_Types_ModuleId __mid;
};

 
struct xdc_runtime_Types_SysFxns2 {
    xdc_Ptr (*__create)(xdc_Ptr, xdc_SizeT, xdc_Ptr, const xdc_UChar*, xdc_SizeT, xdc_runtime_Error_Block*);
    void (*__delete)(xdc_Ptr);
    xdc_runtime_Types_Label *(*__label)(xdc_Ptr, xdc_runtime_Types_Label*);
    xdc_runtime_Types_ModuleId __mid;
};




 

 
typedef xdc_Bits32 CT__xdc_runtime_Types_Module__diagsEnabled;
extern  const CT__xdc_runtime_Types_Module__diagsEnabled xdc_runtime_Types_Module__diagsEnabled__C;

 
typedef xdc_Bits32 CT__xdc_runtime_Types_Module__diagsIncluded;
extern  const CT__xdc_runtime_Types_Module__diagsIncluded xdc_runtime_Types_Module__diagsIncluded__C;

 
typedef xdc_Bits16 *CT__xdc_runtime_Types_Module__diagsMask;
extern  const CT__xdc_runtime_Types_Module__diagsMask xdc_runtime_Types_Module__diagsMask__C;

 
typedef xdc_Ptr CT__xdc_runtime_Types_Module__gateObj;
extern  const CT__xdc_runtime_Types_Module__gateObj xdc_runtime_Types_Module__gateObj__C;

 
typedef xdc_Ptr CT__xdc_runtime_Types_Module__gatePrms;
extern  const CT__xdc_runtime_Types_Module__gatePrms xdc_runtime_Types_Module__gatePrms__C;

 
typedef xdc_runtime_Types_ModuleId CT__xdc_runtime_Types_Module__id;
extern  const CT__xdc_runtime_Types_Module__id xdc_runtime_Types_Module__id__C;

 
typedef xdc_Bool CT__xdc_runtime_Types_Module__loggerDefined;
extern  const CT__xdc_runtime_Types_Module__loggerDefined xdc_runtime_Types_Module__loggerDefined__C;

 
typedef xdc_Ptr CT__xdc_runtime_Types_Module__loggerObj;
extern  const CT__xdc_runtime_Types_Module__loggerObj xdc_runtime_Types_Module__loggerObj__C;

 
typedef xdc_runtime_Types_LoggerFxn0 CT__xdc_runtime_Types_Module__loggerFxn0;
extern  const CT__xdc_runtime_Types_Module__loggerFxn0 xdc_runtime_Types_Module__loggerFxn0__C;

 
typedef xdc_runtime_Types_LoggerFxn1 CT__xdc_runtime_Types_Module__loggerFxn1;
extern  const CT__xdc_runtime_Types_Module__loggerFxn1 xdc_runtime_Types_Module__loggerFxn1__C;

 
typedef xdc_runtime_Types_LoggerFxn2 CT__xdc_runtime_Types_Module__loggerFxn2;
extern  const CT__xdc_runtime_Types_Module__loggerFxn2 xdc_runtime_Types_Module__loggerFxn2__C;

 
typedef xdc_runtime_Types_LoggerFxn4 CT__xdc_runtime_Types_Module__loggerFxn4;
extern  const CT__xdc_runtime_Types_Module__loggerFxn4 xdc_runtime_Types_Module__loggerFxn4__C;

 
typedef xdc_runtime_Types_LoggerFxn8 CT__xdc_runtime_Types_Module__loggerFxn8;
extern  const CT__xdc_runtime_Types_Module__loggerFxn8 xdc_runtime_Types_Module__loggerFxn8__C;

 
typedef xdc_Bool (*CT__xdc_runtime_Types_Module__startupDoneFxn)(void);
extern  const CT__xdc_runtime_Types_Module__startupDoneFxn xdc_runtime_Types_Module__startupDoneFxn__C;

 
typedef xdc_Int CT__xdc_runtime_Types_Object__count;
extern  const CT__xdc_runtime_Types_Object__count xdc_runtime_Types_Object__count__C;

 
typedef xdc_runtime_IHeap_Handle CT__xdc_runtime_Types_Object__heap;
extern  const CT__xdc_runtime_Types_Object__heap xdc_runtime_Types_Object__heap__C;

 
typedef xdc_SizeT CT__xdc_runtime_Types_Object__sizeof;
extern  const CT__xdc_runtime_Types_Object__sizeof xdc_runtime_Types_Object__sizeof__C;

 
typedef xdc_Ptr CT__xdc_runtime_Types_Object__table;
extern  const CT__xdc_runtime_Types_Object__table xdc_runtime_Types_Object__table__C;




 

 


 

extern xdc_Bool xdc_runtime_Types_Module__startupDone__S( void );




 

 


 


 


 
static inline CT__xdc_runtime_Types_Module__id xdc_runtime_Types_Module_id( void ) 
{
    return xdc_runtime_Types_Module__id__C;
}

 
static inline xdc_Bool xdc_runtime_Types_Module_hasMask( void ) 
{
    return xdc_runtime_Types_Module__diagsMask__C != 0;
}

 
static inline xdc_Bits16 xdc_runtime_Types_Module_getMask( void ) 
{
    return xdc_runtime_Types_Module__diagsMask__C != 0 ? *xdc_runtime_Types_Module__diagsMask__C : 0;
}

 
static inline void xdc_runtime_Types_Module_setMask( xdc_Bits16 mask ) 
{
    if (xdc_runtime_Types_Module__diagsMask__C != 0) *xdc_runtime_Types_Module__diagsMask__C = mask;
}




 

#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types__epilogue.h"











 



 



 





 





 



 

#line 341 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"










 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 56 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/ISystemSupport.h"





 













 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 52 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/ISystemSupport.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"





 













 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 52 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 55 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"





 














 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 53 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"





 














 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 53 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"





 




 

 
struct xdc_runtime_IInstance_Params {
    size_t __size;
    xdc_String name;
};




 

 
struct xdc_runtime_IInstance_Fxns__ {
    xdc_runtime_Types_Base* __base;
    const xdc_runtime_Types_SysFxns2 *__sysp;
    xdc_runtime_Types_SysFxns2 __sfxns;
};

 
extern const xdc_runtime_Types_Base xdc_runtime_IInstance_Interface__BASE__C;




 

 

extern xdc_runtime_IInstance_Handle xdc_runtime_IInstance_create(xdc_runtime_IInstance_Module, const xdc_runtime_IInstance_Params *, xdc_runtime_Error_Block *__eb);

 

extern void xdc_runtime_IInstance_delete(xdc_runtime_IInstance_Handle *);

 
static inline xdc_runtime_IInstance_Module xdc_runtime_IInstance_Handle_to_Module( xdc_runtime_IInstance_Handle inst )
{
    return inst->__fxns;
}

 
static inline xdc_runtime_Types_Label *xdc_runtime_IInstance_Handle_label( xdc_runtime_IInstance_Handle inst, xdc_runtime_Types_Label *lab )
{
    return inst->__fxns->__sysp->__label(inst, lab);
}

 
static inline xdc_runtime_Types_ModuleId xdc_runtime_IInstance_Module_id( xdc_runtime_IInstance_Module mod )
{
    return mod->__sysp->__mid;
}




 




 










 

#line 155 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"


 

#line 54 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"





 













 




 

#line 104 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"




 

#line 121 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"


 

#line 57 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"





 















 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 54 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error__prologue.h"











 
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Main.h"





 















 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 54 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Main.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"





 













 




 

#line 104 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"




 

#line 121 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"


 

#line 57 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Main.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IGateProvider.h"





 














 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 53 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IGateProvider.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"





 














 




 

#line 132 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"




 

#line 155 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"


 

#line 54 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IGateProvider.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"





 













 




 

#line 104 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"




 

#line 121 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"


 

#line 57 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IGateProvider.h"




 

 


 





 

 
struct xdc_runtime_IGateProvider_Params {
    size_t __size;
    const void *__self;
    void *__fxns;
    xdc_runtime_IInstance_Params *instance;
};




 

 
struct xdc_runtime_IGateProvider_Fxns__ {
    xdc_runtime_Types_Base* __base;
    const xdc_runtime_Types_SysFxns2 *__sysp;
    xdc_Bool (*query)(xdc_Int);
    xdc_IArg (*enter)(void*);
    void (*leave)(void*, xdc_IArg);
    xdc_runtime_Types_SysFxns2 __sfxns;
};

 
extern const xdc_runtime_Types_Base xdc_runtime_IGateProvider_Interface__BASE__C;




 

 

extern xdc_runtime_IGateProvider_Handle xdc_runtime_IGateProvider_create(xdc_runtime_IGateProvider_Module, const xdc_runtime_IGateProvider_Params *, xdc_runtime_Error_Block *__eb);

 

extern void xdc_runtime_IGateProvider_delete(xdc_runtime_IGateProvider_Handle *);

 
static inline xdc_runtime_IGateProvider_Module xdc_runtime_IGateProvider_Handle_to_Module( xdc_runtime_IGateProvider_Handle inst )
{
    return inst->__fxns;
}

 
static inline xdc_runtime_Types_Label *xdc_runtime_IGateProvider_Handle_label( xdc_runtime_IGateProvider_Handle inst, xdc_runtime_Types_Label *lab )
{
    return inst->__fxns->__sysp->__label(inst, lab);
}

 
static inline xdc_runtime_Types_ModuleId xdc_runtime_IGateProvider_Module_id( xdc_runtime_IGateProvider_Module mod )
{
    return mod->__sysp->__mid;
}

 
static inline xdc_Bool xdc_runtime_IGateProvider_query( xdc_runtime_IGateProvider_Module __inst, xdc_Int qual )
{
    return __inst->query(qual);
}

 
static inline xdc_IArg xdc_runtime_IGateProvider_enter( xdc_runtime_IGateProvider_Handle __inst )
{
    return __inst->__fxns->enter((void*)__inst);
}

 
static inline void xdc_runtime_IGateProvider_leave( xdc_runtime_IGateProvider_Handle __inst, xdc_IArg key )
{
    __inst->__fxns->leave((void*)__inst, key);
}




 






 

 
typedef xdc_Bool (*xdc_runtime_IGateProvider_query_FxnT)(xdc_Int);
static inline xdc_runtime_IGateProvider_query_FxnT xdc_runtime_IGateProvider_query_fxnP( xdc_runtime_IGateProvider_Module __inst )
{
    return (xdc_runtime_IGateProvider_query_FxnT)__inst->query;
}

 
typedef xdc_IArg (*xdc_runtime_IGateProvider_enter_FxnT)(void *);
static inline xdc_runtime_IGateProvider_enter_FxnT xdc_runtime_IGateProvider_enter_fxnP( xdc_runtime_IGateProvider_Handle __inst )
{
    return (xdc_runtime_IGateProvider_enter_FxnT)__inst->__fxns->enter;
}

 
typedef void (*xdc_runtime_IGateProvider_leave_FxnT)(void *, xdc_IArg);
static inline xdc_runtime_IGateProvider_leave_FxnT xdc_runtime_IGateProvider_leave_fxnP( xdc_runtime_IGateProvider_Handle __inst )
{
    return (xdc_runtime_IGateProvider_leave_FxnT)__inst->__fxns->leave;
}




 










 

#line 227 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IGateProvider.h"


 

#line 58 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Main.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/Main_Module_GateProxy.h"





 
















 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 55 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/Main_Module_GateProxy.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"





 














 




 

#line 132 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"




 

#line 155 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"


 

#line 56 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/Main_Module_GateProxy.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IGateProvider.h"





 














 




 

#line 192 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IGateProvider.h"




 

#line 227 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IGateProvider.h"


 

#line 59 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/Main_Module_GateProxy.h"




 

 


 





 

 
typedef xdc_Bits32 CT__xdc_runtime_Main_Module_GateProxy_Module__diagsEnabled;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__diagsEnabled xdc_runtime_Main_Module_GateProxy_Module__diagsEnabled__C;

 
typedef xdc_Bits32 CT__xdc_runtime_Main_Module_GateProxy_Module__diagsIncluded;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__diagsIncluded xdc_runtime_Main_Module_GateProxy_Module__diagsIncluded__C;

 
typedef xdc_Bits16 *CT__xdc_runtime_Main_Module_GateProxy_Module__diagsMask;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__diagsMask xdc_runtime_Main_Module_GateProxy_Module__diagsMask__C;

 
typedef xdc_Ptr CT__xdc_runtime_Main_Module_GateProxy_Module__gateObj;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__gateObj xdc_runtime_Main_Module_GateProxy_Module__gateObj__C;

 
typedef xdc_Ptr CT__xdc_runtime_Main_Module_GateProxy_Module__gatePrms;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__gatePrms xdc_runtime_Main_Module_GateProxy_Module__gatePrms__C;

 
typedef xdc_runtime_Types_ModuleId CT__xdc_runtime_Main_Module_GateProxy_Module__id;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__id xdc_runtime_Main_Module_GateProxy_Module__id__C;

 
typedef xdc_Bool CT__xdc_runtime_Main_Module_GateProxy_Module__loggerDefined;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__loggerDefined xdc_runtime_Main_Module_GateProxy_Module__loggerDefined__C;

 
typedef xdc_Ptr CT__xdc_runtime_Main_Module_GateProxy_Module__loggerObj;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__loggerObj xdc_runtime_Main_Module_GateProxy_Module__loggerObj__C;

 
typedef xdc_runtime_Types_LoggerFxn0 CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn0;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn0 xdc_runtime_Main_Module_GateProxy_Module__loggerFxn0__C;

 
typedef xdc_runtime_Types_LoggerFxn1 CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn1;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn1 xdc_runtime_Main_Module_GateProxy_Module__loggerFxn1__C;

 
typedef xdc_runtime_Types_LoggerFxn2 CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn2;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn2 xdc_runtime_Main_Module_GateProxy_Module__loggerFxn2__C;

 
typedef xdc_runtime_Types_LoggerFxn4 CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn4;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn4 xdc_runtime_Main_Module_GateProxy_Module__loggerFxn4__C;

 
typedef xdc_runtime_Types_LoggerFxn8 CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn8;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__loggerFxn8 xdc_runtime_Main_Module_GateProxy_Module__loggerFxn8__C;

 
typedef xdc_Bool (*CT__xdc_runtime_Main_Module_GateProxy_Module__startupDoneFxn)(void);
extern  const CT__xdc_runtime_Main_Module_GateProxy_Module__startupDoneFxn xdc_runtime_Main_Module_GateProxy_Module__startupDoneFxn__C;

 
typedef xdc_Int CT__xdc_runtime_Main_Module_GateProxy_Object__count;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Object__count xdc_runtime_Main_Module_GateProxy_Object__count__C;

 
typedef xdc_runtime_IHeap_Handle CT__xdc_runtime_Main_Module_GateProxy_Object__heap;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Object__heap xdc_runtime_Main_Module_GateProxy_Object__heap__C;

 
typedef xdc_SizeT CT__xdc_runtime_Main_Module_GateProxy_Object__sizeof;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Object__sizeof xdc_runtime_Main_Module_GateProxy_Object__sizeof__C;

 
typedef xdc_Ptr CT__xdc_runtime_Main_Module_GateProxy_Object__table;
extern  const CT__xdc_runtime_Main_Module_GateProxy_Object__table xdc_runtime_Main_Module_GateProxy_Object__table__C;




 

 
struct xdc_runtime_Main_Module_GateProxy_Params {
    size_t __size;
    const void *__self;
    void *__fxns;
    xdc_runtime_IInstance_Params *instance;
    xdc_runtime_IInstance_Params __iprms;
};

 
struct xdc_runtime_Main_Module_GateProxy_Struct {
    const xdc_runtime_Main_Module_GateProxy_Fxns__ *__fxns;
    xdc_runtime_Types_CordAddr __name;
};




 

 
struct xdc_runtime_Main_Module_GateProxy_Fxns__ {
    xdc_runtime_Types_Base* __base;
    const xdc_runtime_Types_SysFxns2 *__sysp;
    xdc_Bool (*query)(xdc_Int);
    xdc_IArg (*enter)(xdc_runtime_Main_Module_GateProxy_Handle);
    void (*leave)(xdc_runtime_Main_Module_GateProxy_Handle, xdc_IArg);
    xdc_runtime_Types_SysFxns2 __sfxns;
};

 
extern const xdc_runtime_Main_Module_GateProxy_Fxns__ xdc_runtime_Main_Module_GateProxy_Module__FXNS__C;




 

 


 

extern xdc_runtime_Types_Label *xdc_runtime_Main_Module_GateProxy_Handle__label__S( xdc_Ptr obj, xdc_runtime_Types_Label *lab );

 

extern xdc_Bool xdc_runtime_Main_Module_GateProxy_Module__startupDone__S( void );

 

extern xdc_Ptr xdc_runtime_Main_Module_GateProxy_Object__create__S( xdc_Ptr __oa, xdc_SizeT __osz, xdc_Ptr __aa, const xdc_UChar *__pa, xdc_SizeT __psz, xdc_runtime_Error_Block *__eb );

 

extern xdc_runtime_Main_Module_GateProxy_Handle xdc_runtime_Main_Module_GateProxy_create( const xdc_runtime_Main_Module_GateProxy_Params *__prms, xdc_runtime_Error_Block *__eb );

 

extern void xdc_runtime_Main_Module_GateProxy_Object__delete__S( xdc_Ptr instp );

 

extern void xdc_runtime_Main_Module_GateProxy_delete(xdc_runtime_Main_Module_GateProxy_Handle *instp);

 

extern void xdc_runtime_Main_Module_GateProxy_Object__destruct__S( xdc_Ptr objp );

 

extern xdc_Ptr xdc_runtime_Main_Module_GateProxy_Object__get__S( xdc_Ptr oarr, xdc_Int i );

 

extern xdc_Ptr xdc_runtime_Main_Module_GateProxy_Object__first__S( void );

 

extern xdc_Ptr xdc_runtime_Main_Module_GateProxy_Object__next__S( xdc_Ptr obj );

 

extern void xdc_runtime_Main_Module_GateProxy_Params__init__S( xdc_Ptr dst, const void *src, xdc_SizeT psz, xdc_SizeT isz );

 

extern xdc_Bool xdc_runtime_Main_Module_GateProxy_Proxy__abstract__S( void );

 

extern xdc_Ptr xdc_runtime_Main_Module_GateProxy_Proxy__delegate__S( void );

 


extern xdc_Bool xdc_runtime_Main_Module_GateProxy_query__E( xdc_Int qual );

 


extern xdc_IArg xdc_runtime_Main_Module_GateProxy_enter__E( xdc_runtime_Main_Module_GateProxy_Handle __inst );

 


extern void xdc_runtime_Main_Module_GateProxy_leave__E( xdc_runtime_Main_Module_GateProxy_Handle __inst, xdc_IArg key );




 

 
static inline xdc_runtime_IGateProvider_Module xdc_runtime_Main_Module_GateProxy_Module_upCast( void )
{
    return (xdc_runtime_IGateProvider_Module)xdc_runtime_Main_Module_GateProxy_Proxy__delegate__S();
}

 


 
static inline xdc_runtime_IGateProvider_Handle xdc_runtime_Main_Module_GateProxy_Handle_upCast( xdc_runtime_Main_Module_GateProxy_Handle i )
{
    return (xdc_runtime_IGateProvider_Handle)i;
}

 


 
static inline xdc_runtime_Main_Module_GateProxy_Handle xdc_runtime_Main_Module_GateProxy_Handle_downCast( xdc_runtime_IGateProvider_Handle i )
{
    xdc_runtime_IGateProvider_Handle i2 = (xdc_runtime_IGateProvider_Handle)i;
if (xdc_runtime_Main_Module_GateProxy_Proxy__abstract__S()) return (xdc_runtime_Main_Module_GateProxy_Handle)i;
    return (void*)i2->__fxns == (void*)xdc_runtime_Main_Module_GateProxy_Proxy__delegate__S() ? (xdc_runtime_Main_Module_GateProxy_Handle)i : 0;
}

 





 

 


 


 


 
static inline CT__xdc_runtime_Main_Module_GateProxy_Module__id xdc_runtime_Main_Module_GateProxy_Module_id( void ) 
{
    return xdc_runtime_Main_Module_GateProxy_Module__id__C;
}

 


 


 
static inline void xdc_runtime_Main_Module_GateProxy_Params_init( xdc_runtime_Main_Module_GateProxy_Params *prms ) 
{
    if (prms) {
        xdc_runtime_Main_Module_GateProxy_Params__init__S(prms, 0, sizeof(xdc_runtime_Main_Module_GateProxy_Params), sizeof(xdc_runtime_IInstance_Params));
    }
}

 
static inline void xdc_runtime_Main_Module_GateProxy_Params_copy(xdc_runtime_Main_Module_GateProxy_Params *dst, const xdc_runtime_Main_Module_GateProxy_Params *src) 
{
    if (dst) {
        xdc_runtime_Main_Module_GateProxy_Params__init__S(dst, (const void *)src, sizeof(xdc_runtime_Main_Module_GateProxy_Params), sizeof(xdc_runtime_IInstance_Params));
    }
}




 










 

#line 59 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Main.h"




 




 




 

 
typedef xdc_Bits32 CT__xdc_runtime_Main_Module__diagsEnabled;
extern  const CT__xdc_runtime_Main_Module__diagsEnabled xdc_runtime_Main_Module__diagsEnabled__C;

 
typedef xdc_Bits32 CT__xdc_runtime_Main_Module__diagsIncluded;
extern  const CT__xdc_runtime_Main_Module__diagsIncluded xdc_runtime_Main_Module__diagsIncluded__C;

 
typedef xdc_Bits16 *CT__xdc_runtime_Main_Module__diagsMask;
extern  const CT__xdc_runtime_Main_Module__diagsMask xdc_runtime_Main_Module__diagsMask__C;

 
typedef xdc_Ptr CT__xdc_runtime_Main_Module__gateObj;
extern  const CT__xdc_runtime_Main_Module__gateObj xdc_runtime_Main_Module__gateObj__C;

 
typedef xdc_Ptr CT__xdc_runtime_Main_Module__gatePrms;
extern  const CT__xdc_runtime_Main_Module__gatePrms xdc_runtime_Main_Module__gatePrms__C;

 
typedef xdc_runtime_Types_ModuleId CT__xdc_runtime_Main_Module__id;
extern  const CT__xdc_runtime_Main_Module__id xdc_runtime_Main_Module__id__C;

 
typedef xdc_Bool CT__xdc_runtime_Main_Module__loggerDefined;
extern  const CT__xdc_runtime_Main_Module__loggerDefined xdc_runtime_Main_Module__loggerDefined__C;

 
typedef xdc_Ptr CT__xdc_runtime_Main_Module__loggerObj;
extern  const CT__xdc_runtime_Main_Module__loggerObj xdc_runtime_Main_Module__loggerObj__C;

 
typedef xdc_runtime_Types_LoggerFxn0 CT__xdc_runtime_Main_Module__loggerFxn0;
extern  const CT__xdc_runtime_Main_Module__loggerFxn0 xdc_runtime_Main_Module__loggerFxn0__C;

 
typedef xdc_runtime_Types_LoggerFxn1 CT__xdc_runtime_Main_Module__loggerFxn1;
extern  const CT__xdc_runtime_Main_Module__loggerFxn1 xdc_runtime_Main_Module__loggerFxn1__C;

 
typedef xdc_runtime_Types_LoggerFxn2 CT__xdc_runtime_Main_Module__loggerFxn2;
extern  const CT__xdc_runtime_Main_Module__loggerFxn2 xdc_runtime_Main_Module__loggerFxn2__C;

 
typedef xdc_runtime_Types_LoggerFxn4 CT__xdc_runtime_Main_Module__loggerFxn4;
extern  const CT__xdc_runtime_Main_Module__loggerFxn4 xdc_runtime_Main_Module__loggerFxn4__C;

 
typedef xdc_runtime_Types_LoggerFxn8 CT__xdc_runtime_Main_Module__loggerFxn8;
extern  const CT__xdc_runtime_Main_Module__loggerFxn8 xdc_runtime_Main_Module__loggerFxn8__C;

 
typedef xdc_Bool (*CT__xdc_runtime_Main_Module__startupDoneFxn)(void);
extern  const CT__xdc_runtime_Main_Module__startupDoneFxn xdc_runtime_Main_Module__startupDoneFxn__C;

 
typedef xdc_Int CT__xdc_runtime_Main_Object__count;
extern  const CT__xdc_runtime_Main_Object__count xdc_runtime_Main_Object__count__C;

 
typedef xdc_runtime_IHeap_Handle CT__xdc_runtime_Main_Object__heap;
extern  const CT__xdc_runtime_Main_Object__heap xdc_runtime_Main_Object__heap__C;

 
typedef xdc_SizeT CT__xdc_runtime_Main_Object__sizeof;
extern  const CT__xdc_runtime_Main_Object__sizeof xdc_runtime_Main_Object__sizeof__C;

 
typedef xdc_Ptr CT__xdc_runtime_Main_Object__table;
extern  const CT__xdc_runtime_Main_Object__table xdc_runtime_Main_Object__table__C;




 

 


 

extern xdc_Bool xdc_runtime_Main_Module__startupDone__S( void );




 

 


 


 


 
static inline CT__xdc_runtime_Main_Module__id xdc_runtime_Main_Module_id( void ) 
{
    return xdc_runtime_Main_Module__id__C;
}

 
static inline xdc_Bool xdc_runtime_Main_Module_hasMask( void ) 
{
    return xdc_runtime_Main_Module__diagsMask__C != 0;
}

 
static inline xdc_Bits16 xdc_runtime_Main_Module_getMask( void ) 
{
    return xdc_runtime_Main_Module__diagsMask__C != 0 ? *xdc_runtime_Main_Module__diagsMask__C : 0;
}

 
static inline void xdc_runtime_Main_Module_setMask( xdc_Bits16 mask ) 
{
    if (xdc_runtime_Main_Module__diagsMask__C != 0) *xdc_runtime_Main_Module__diagsMask__C = mask;
}




 










 

#line 222 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Main.h"



 

#line 248 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Main.h"


 

#line 14 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error__prologue.h"



 
typedef xdc_Bits32 xdc_runtime_Error_Id;


















 








 

#line 55 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"





 













 




 

#line 104 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"




 

#line 121 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"


 

#line 58 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 59 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"




 

 
enum xdc_runtime_Error_Policy {
    xdc_runtime_Error_TERMINATE,
    xdc_runtime_Error_UNWIND
};
typedef enum xdc_runtime_Error_Policy xdc_runtime_Error_Policy;

 

 
typedef void (*xdc_runtime_Error_HookFxn)(xdc_runtime_Error_Block*);

 


 
typedef xdc_IArg __T1_xdc_runtime_Error_Data__arg;
typedef xdc_IArg __ARRAY1_xdc_runtime_Error_Data__arg[2];
typedef __ARRAY1_xdc_runtime_Error_Data__arg __TA_xdc_runtime_Error_Data__arg;
struct xdc_runtime_Error_Data {
    __TA_xdc_runtime_Error_Data__arg arg;
};

 
typedef xdc_IArg __T1_xdc_runtime_Error_Block__xtra;
typedef xdc_IArg __ARRAY1_xdc_runtime_Error_Block__xtra[4];
typedef __ARRAY1_xdc_runtime_Error_Block__xtra __TA_xdc_runtime_Error_Block__xtra;
struct xdc_runtime_Error_Block {
    xdc_UInt16 unused;
    xdc_runtime_Error_Data data;
    xdc_runtime_Error_Id id;
    xdc_String msg;
    xdc_runtime_Types_Site site;
    __TA_xdc_runtime_Error_Block__xtra xtra;
};

 
typedef void (*xdc_runtime_Error_PolicyFxn)(xdc_runtime_Error_Block*, xdc_runtime_Types_ModuleId, xdc_CString, xdc_Int, xdc_runtime_Error_Id, xdc_IArg, xdc_IArg);




 




 

 
typedef xdc_Bits32 CT__xdc_runtime_Error_Module__diagsEnabled;
extern  const CT__xdc_runtime_Error_Module__diagsEnabled xdc_runtime_Error_Module__diagsEnabled__C;

 
typedef xdc_Bits32 CT__xdc_runtime_Error_Module__diagsIncluded;
extern  const CT__xdc_runtime_Error_Module__diagsIncluded xdc_runtime_Error_Module__diagsIncluded__C;

 
typedef xdc_Bits16 *CT__xdc_runtime_Error_Module__diagsMask;
extern  const CT__xdc_runtime_Error_Module__diagsMask xdc_runtime_Error_Module__diagsMask__C;

 
typedef xdc_Ptr CT__xdc_runtime_Error_Module__gateObj;
extern  const CT__xdc_runtime_Error_Module__gateObj xdc_runtime_Error_Module__gateObj__C;

 
typedef xdc_Ptr CT__xdc_runtime_Error_Module__gatePrms;
extern  const CT__xdc_runtime_Error_Module__gatePrms xdc_runtime_Error_Module__gatePrms__C;

 
typedef xdc_runtime_Types_ModuleId CT__xdc_runtime_Error_Module__id;
extern  const CT__xdc_runtime_Error_Module__id xdc_runtime_Error_Module__id__C;

 
typedef xdc_Bool CT__xdc_runtime_Error_Module__loggerDefined;
extern  const CT__xdc_runtime_Error_Module__loggerDefined xdc_runtime_Error_Module__loggerDefined__C;

 
typedef xdc_Ptr CT__xdc_runtime_Error_Module__loggerObj;
extern  const CT__xdc_runtime_Error_Module__loggerObj xdc_runtime_Error_Module__loggerObj__C;

 
typedef xdc_runtime_Types_LoggerFxn0 CT__xdc_runtime_Error_Module__loggerFxn0;
extern  const CT__xdc_runtime_Error_Module__loggerFxn0 xdc_runtime_Error_Module__loggerFxn0__C;

 
typedef xdc_runtime_Types_LoggerFxn1 CT__xdc_runtime_Error_Module__loggerFxn1;
extern  const CT__xdc_runtime_Error_Module__loggerFxn1 xdc_runtime_Error_Module__loggerFxn1__C;

 
typedef xdc_runtime_Types_LoggerFxn2 CT__xdc_runtime_Error_Module__loggerFxn2;
extern  const CT__xdc_runtime_Error_Module__loggerFxn2 xdc_runtime_Error_Module__loggerFxn2__C;

 
typedef xdc_runtime_Types_LoggerFxn4 CT__xdc_runtime_Error_Module__loggerFxn4;
extern  const CT__xdc_runtime_Error_Module__loggerFxn4 xdc_runtime_Error_Module__loggerFxn4__C;

 
typedef xdc_runtime_Types_LoggerFxn8 CT__xdc_runtime_Error_Module__loggerFxn8;
extern  const CT__xdc_runtime_Error_Module__loggerFxn8 xdc_runtime_Error_Module__loggerFxn8__C;

 
typedef xdc_Bool (*CT__xdc_runtime_Error_Module__startupDoneFxn)(void);
extern  const CT__xdc_runtime_Error_Module__startupDoneFxn xdc_runtime_Error_Module__startupDoneFxn__C;

 
typedef xdc_Int CT__xdc_runtime_Error_Object__count;
extern  const CT__xdc_runtime_Error_Object__count xdc_runtime_Error_Object__count__C;

 
typedef xdc_runtime_IHeap_Handle CT__xdc_runtime_Error_Object__heap;
extern  const CT__xdc_runtime_Error_Object__heap xdc_runtime_Error_Object__heap__C;

 
typedef xdc_SizeT CT__xdc_runtime_Error_Object__sizeof;
extern  const CT__xdc_runtime_Error_Object__sizeof xdc_runtime_Error_Object__sizeof__C;

 
typedef xdc_Ptr CT__xdc_runtime_Error_Object__table;
extern  const CT__xdc_runtime_Error_Object__table xdc_runtime_Error_Object__table__C;

 

typedef xdc_runtime_Error_PolicyFxn CT__xdc_runtime_Error_policyFxn;
extern  const CT__xdc_runtime_Error_policyFxn xdc_runtime_Error_policyFxn__C;

 

typedef xdc_runtime_Error_Id CT__xdc_runtime_Error_E_generic;
extern  const CT__xdc_runtime_Error_E_generic xdc_runtime_Error_E_generic__C;

 

typedef xdc_runtime_Error_Id CT__xdc_runtime_Error_E_memory;
extern  const CT__xdc_runtime_Error_E_memory xdc_runtime_Error_E_memory__C;

 

typedef xdc_runtime_Error_Id CT__xdc_runtime_Error_E_msgCode;
extern  const CT__xdc_runtime_Error_E_msgCode xdc_runtime_Error_E_msgCode__C;

 

typedef xdc_runtime_Error_Policy CT__xdc_runtime_Error_policy;
extern  const CT__xdc_runtime_Error_policy xdc_runtime_Error_policy__C;

 

typedef xdc_runtime_Error_HookFxn CT__xdc_runtime_Error_raiseHook;
extern  const CT__xdc_runtime_Error_raiseHook xdc_runtime_Error_raiseHook__C;

 




typedef xdc_UInt16 CT__xdc_runtime_Error_maxDepth;
extern  const CT__xdc_runtime_Error_maxDepth xdc_runtime_Error_maxDepth__C;





 

 


 

extern xdc_Bool xdc_runtime_Error_Module__startupDone__S( void );

 


extern xdc_Bool xdc_runtime_Error_check__E( xdc_runtime_Error_Block *eb );

 


extern xdc_runtime_Error_Data *xdc_runtime_Error_getData__E( xdc_runtime_Error_Block *eb );

 


extern xdc_UInt16 xdc_runtime_Error_getCode__E( xdc_runtime_Error_Block *eb );

 


extern xdc_runtime_Error_Id xdc_runtime_Error_getId__E( xdc_runtime_Error_Block *eb );

 


extern xdc_String xdc_runtime_Error_getMsg__E( xdc_runtime_Error_Block *eb );

 


extern xdc_runtime_Types_Site *xdc_runtime_Error_getSite__E( xdc_runtime_Error_Block *eb );

 


extern void xdc_runtime_Error_init__E( xdc_runtime_Error_Block *eb );

 


extern void xdc_runtime_Error_print__E( xdc_runtime_Error_Block *eb );

 


extern void xdc_runtime_Error_policyDefault__E( xdc_runtime_Error_Block *eb, xdc_runtime_Types_ModuleId mod, xdc_CString file, xdc_Int line, xdc_runtime_Error_Id id, xdc_IArg arg1, xdc_IArg arg2 );

 


extern void xdc_runtime_Error_policySpin__E( xdc_runtime_Error_Block *eb, xdc_runtime_Types_ModuleId mod, xdc_CString file, xdc_Int line, xdc_runtime_Error_Id id, xdc_IArg arg1, xdc_IArg arg2 );

 


extern void xdc_runtime_Error_raiseX__E( xdc_runtime_Error_Block *eb, xdc_runtime_Types_ModuleId mod, xdc_CString file, xdc_Int line, xdc_runtime_Error_Id id, xdc_IArg arg1, xdc_IArg arg2 );

 


extern void xdc_runtime_Error_setX__E( xdc_runtime_Error_Block *eb, xdc_runtime_Types_ModuleId mod, xdc_CString file, xdc_Int line, xdc_runtime_Error_Id id, xdc_IArg arg1, xdc_IArg arg2 );




 

 


 


 


 
static inline CT__xdc_runtime_Error_Module__id xdc_runtime_Error_Module_id( void ) 
{
    return xdc_runtime_Error_Module__id__C;
}

 
static inline xdc_Bool xdc_runtime_Error_Module_hasMask( void ) 
{
    return xdc_runtime_Error_Module__diagsMask__C != 0;
}

 
static inline xdc_Bits16 xdc_runtime_Error_Module_getMask( void ) 
{
    return xdc_runtime_Error_Module__diagsMask__C != 0 ? *xdc_runtime_Error_Module__diagsMask__C : 0;
}

 
static inline void xdc_runtime_Error_Module_setMask( xdc_Bits16 mask ) 
{
    if (xdc_runtime_Error_Module__diagsMask__C != 0) *xdc_runtime_Error_Module__diagsMask__C = mask;
}




 

#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error__epilogue.h"











 








 






 





 




 

#line 341 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"










 

#line 369 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"




 

#line 425 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"


 

#line 58 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.h"





 















 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 54 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"





 













 




 

#line 104 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"




 

#line 121 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"


 

#line 57 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"





 














 




 

#line 202 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"




 

#line 238 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"


 

#line 58 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"




 

#line 369 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"




 

#line 425 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"


 

#line 59 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/Memory_HeapProxy.h"





 
















 




 




















 



#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"




 

#line 362 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"



 

#line 423 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Types.h"


 

#line 55 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/Memory_HeapProxy.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"





 














 




 

#line 132 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"




 

#line 155 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IInstance.h"


 

#line 56 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/Memory_HeapProxy.h"


#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"





 














 




 

#line 202 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"




 

#line 238 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"


 

#line 59 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/package/Memory_HeapProxy.h"




 




 

 
typedef xdc_Bits32 CT__xdc_runtime_Memory_HeapProxy_Module__diagsEnabled;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__diagsEnabled xdc_runtime_Memory_HeapProxy_Module__diagsEnabled__C;

 
typedef xdc_Bits32 CT__xdc_runtime_Memory_HeapProxy_Module__diagsIncluded;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__diagsIncluded xdc_runtime_Memory_HeapProxy_Module__diagsIncluded__C;

 
typedef xdc_Bits16 *CT__xdc_runtime_Memory_HeapProxy_Module__diagsMask;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__diagsMask xdc_runtime_Memory_HeapProxy_Module__diagsMask__C;

 
typedef xdc_Ptr CT__xdc_runtime_Memory_HeapProxy_Module__gateObj;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__gateObj xdc_runtime_Memory_HeapProxy_Module__gateObj__C;

 
typedef xdc_Ptr CT__xdc_runtime_Memory_HeapProxy_Module__gatePrms;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__gatePrms xdc_runtime_Memory_HeapProxy_Module__gatePrms__C;

 
typedef xdc_runtime_Types_ModuleId CT__xdc_runtime_Memory_HeapProxy_Module__id;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__id xdc_runtime_Memory_HeapProxy_Module__id__C;

 
typedef xdc_Bool CT__xdc_runtime_Memory_HeapProxy_Module__loggerDefined;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__loggerDefined xdc_runtime_Memory_HeapProxy_Module__loggerDefined__C;

 
typedef xdc_Ptr CT__xdc_runtime_Memory_HeapProxy_Module__loggerObj;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__loggerObj xdc_runtime_Memory_HeapProxy_Module__loggerObj__C;

 
typedef xdc_runtime_Types_LoggerFxn0 CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn0;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn0 xdc_runtime_Memory_HeapProxy_Module__loggerFxn0__C;

 
typedef xdc_runtime_Types_LoggerFxn1 CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn1;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn1 xdc_runtime_Memory_HeapProxy_Module__loggerFxn1__C;

 
typedef xdc_runtime_Types_LoggerFxn2 CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn2;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn2 xdc_runtime_Memory_HeapProxy_Module__loggerFxn2__C;

 
typedef xdc_runtime_Types_LoggerFxn4 CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn4;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn4 xdc_runtime_Memory_HeapProxy_Module__loggerFxn4__C;

 
typedef xdc_runtime_Types_LoggerFxn8 CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn8;
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__loggerFxn8 xdc_runtime_Memory_HeapProxy_Module__loggerFxn8__C;

 
typedef xdc_Bool (*CT__xdc_runtime_Memory_HeapProxy_Module__startupDoneFxn)(void);
extern  const CT__xdc_runtime_Memory_HeapProxy_Module__startupDoneFxn xdc_runtime_Memory_HeapProxy_Module__startupDoneFxn__C;

 
typedef xdc_Int CT__xdc_runtime_Memory_HeapProxy_Object__count;
extern  const CT__xdc_runtime_Memory_HeapProxy_Object__count xdc_runtime_Memory_HeapProxy_Object__count__C;

 
typedef xdc_runtime_IHeap_Handle CT__xdc_runtime_Memory_HeapProxy_Object__heap;
extern  const CT__xdc_runtime_Memory_HeapProxy_Object__heap xdc_runtime_Memory_HeapProxy_Object__heap__C;

 
typedef xdc_SizeT CT__xdc_runtime_Memory_HeapProxy_Object__sizeof;
extern  const CT__xdc_runtime_Memory_HeapProxy_Object__sizeof xdc_runtime_Memory_HeapProxy_Object__sizeof__C;

 
typedef xdc_Ptr CT__xdc_runtime_Memory_HeapProxy_Object__table;
extern  const CT__xdc_runtime_Memory_HeapProxy_Object__table xdc_runtime_Memory_HeapProxy_Object__table__C;




 

 
struct xdc_runtime_Memory_HeapProxy_Params {
    size_t __size;
    const void *__self;
    void *__fxns;
    xdc_runtime_IInstance_Params *instance;
    xdc_runtime_IInstance_Params __iprms;
};

 
struct xdc_runtime_Memory_HeapProxy_Struct {
    const xdc_runtime_Memory_HeapProxy_Fxns__ *__fxns;
    xdc_runtime_Types_CordAddr __name;
};




 

 
struct xdc_runtime_Memory_HeapProxy_Fxns__ {
    xdc_runtime_Types_Base* __base;
    const xdc_runtime_Types_SysFxns2 *__sysp;
    xdc_Ptr (*alloc)(xdc_runtime_Memory_HeapProxy_Handle, xdc_SizeT, xdc_SizeT, xdc_runtime_Error_Block*);
    void (*free)(xdc_runtime_Memory_HeapProxy_Handle, xdc_Ptr, xdc_SizeT);
    xdc_Bool (*isBlocking)(xdc_runtime_Memory_HeapProxy_Handle);
    void (*getStats)(xdc_runtime_Memory_HeapProxy_Handle, xdc_runtime_Memory_Stats*);
    xdc_runtime_Types_SysFxns2 __sfxns;
};

 
extern const xdc_runtime_Memory_HeapProxy_Fxns__ xdc_runtime_Memory_HeapProxy_Module__FXNS__C;




 

 


 

extern xdc_runtime_Types_Label *xdc_runtime_Memory_HeapProxy_Handle__label__S( xdc_Ptr obj, xdc_runtime_Types_Label *lab );

 

extern xdc_Bool xdc_runtime_Memory_HeapProxy_Module__startupDone__S( void );

 

extern xdc_Ptr xdc_runtime_Memory_HeapProxy_Object__create__S( xdc_Ptr __oa, xdc_SizeT __osz, xdc_Ptr __aa, const xdc_UChar *__pa, xdc_SizeT __psz, xdc_runtime_Error_Block *__eb );

 

extern xdc_runtime_Memory_HeapProxy_Handle xdc_runtime_Memory_HeapProxy_create( const xdc_runtime_Memory_HeapProxy_Params *__prms, xdc_runtime_Error_Block *__eb );

 

extern void xdc_runtime_Memory_HeapProxy_Object__delete__S( xdc_Ptr instp );

 

extern void xdc_runtime_Memory_HeapProxy_delete(xdc_runtime_Memory_HeapProxy_Handle *instp);

 

extern void xdc_runtime_Memory_HeapProxy_Object__destruct__S( xdc_Ptr objp );

 

extern xdc_Ptr xdc_runtime_Memory_HeapProxy_Object__get__S( xdc_Ptr oarr, xdc_Int i );

 

extern xdc_Ptr xdc_runtime_Memory_HeapProxy_Object__first__S( void );

 

extern xdc_Ptr xdc_runtime_Memory_HeapProxy_Object__next__S( xdc_Ptr obj );

 

extern void xdc_runtime_Memory_HeapProxy_Params__init__S( xdc_Ptr dst, const void *src, xdc_SizeT psz, xdc_SizeT isz );

 

extern xdc_Bool xdc_runtime_Memory_HeapProxy_Proxy__abstract__S( void );

 

extern xdc_Ptr xdc_runtime_Memory_HeapProxy_Proxy__delegate__S( void );

 


extern xdc_Ptr xdc_runtime_Memory_HeapProxy_alloc__E( xdc_runtime_Memory_HeapProxy_Handle __inst, xdc_SizeT size, xdc_SizeT align, xdc_runtime_Error_Block *eb );

 


extern void xdc_runtime_Memory_HeapProxy_free__E( xdc_runtime_Memory_HeapProxy_Handle __inst, xdc_Ptr block, xdc_SizeT size );

 


extern xdc_Bool xdc_runtime_Memory_HeapProxy_isBlocking__E( xdc_runtime_Memory_HeapProxy_Handle __inst );

 


extern void xdc_runtime_Memory_HeapProxy_getStats__E( xdc_runtime_Memory_HeapProxy_Handle __inst, xdc_runtime_Memory_Stats *stats );




 

 
static inline xdc_runtime_IHeap_Module xdc_runtime_Memory_HeapProxy_Module_upCast( void )
{
    return (xdc_runtime_IHeap_Module)xdc_runtime_Memory_HeapProxy_Proxy__delegate__S();
}

 


 
static inline xdc_runtime_IHeap_Handle xdc_runtime_Memory_HeapProxy_Handle_upCast( xdc_runtime_Memory_HeapProxy_Handle i )
{
    return (xdc_runtime_IHeap_Handle)i;
}

 


 
static inline xdc_runtime_Memory_HeapProxy_Handle xdc_runtime_Memory_HeapProxy_Handle_downCast( xdc_runtime_IHeap_Handle i )
{
    xdc_runtime_IHeap_Handle i2 = (xdc_runtime_IHeap_Handle)i;
if (xdc_runtime_Memory_HeapProxy_Proxy__abstract__S()) return (xdc_runtime_Memory_HeapProxy_Handle)i;
    return (void*)i2->__fxns == (void*)xdc_runtime_Memory_HeapProxy_Proxy__delegate__S() ? (xdc_runtime_Memory_HeapProxy_Handle)i : 0;
}

 





 

 


 


 


 
static inline CT__xdc_runtime_Memory_HeapProxy_Module__id xdc_runtime_Memory_HeapProxy_Module_id( void ) 
{
    return xdc_runtime_Memory_HeapProxy_Module__id__C;
}

 


 


 
static inline void xdc_runtime_Memory_HeapProxy_Params_init( xdc_runtime_Memory_HeapProxy_Params *prms ) 
{
    if (prms) {
        xdc_runtime_Memory_HeapProxy_Params__init__S(prms, 0, sizeof(xdc_runtime_Memory_HeapProxy_Params), sizeof(xdc_runtime_IInstance_Params));
    }
}

 
static inline void xdc_runtime_Memory_HeapProxy_Params_copy(xdc_runtime_Memory_HeapProxy_Params *dst, const xdc_runtime_Memory_HeapProxy_Params *src) 
{
    if (dst) {
        xdc_runtime_Memory_HeapProxy_Params__init__S(dst, (const void *)src, sizeof(xdc_runtime_Memory_HeapProxy_Params), sizeof(xdc_runtime_IInstance_Params));
    }
}




 










 

#line 60 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.h"




 

 


 
typedef xdc_UArg xdc_runtime_Memory_Size;

 
struct xdc_runtime_Memory_Stats {
    xdc_runtime_Memory_Size totalSize;
    xdc_runtime_Memory_Size totalFreeSize;
    xdc_runtime_Memory_Size largestFreeSize;
};




 




 

 
typedef xdc_Bits32 CT__xdc_runtime_Memory_Module__diagsEnabled;
extern  const CT__xdc_runtime_Memory_Module__diagsEnabled xdc_runtime_Memory_Module__diagsEnabled__C;

 
typedef xdc_Bits32 CT__xdc_runtime_Memory_Module__diagsIncluded;
extern  const CT__xdc_runtime_Memory_Module__diagsIncluded xdc_runtime_Memory_Module__diagsIncluded__C;

 
typedef xdc_Bits16 *CT__xdc_runtime_Memory_Module__diagsMask;
extern  const CT__xdc_runtime_Memory_Module__diagsMask xdc_runtime_Memory_Module__diagsMask__C;

 
typedef xdc_Ptr CT__xdc_runtime_Memory_Module__gateObj;
extern  const CT__xdc_runtime_Memory_Module__gateObj xdc_runtime_Memory_Module__gateObj__C;

 
typedef xdc_Ptr CT__xdc_runtime_Memory_Module__gatePrms;
extern  const CT__xdc_runtime_Memory_Module__gatePrms xdc_runtime_Memory_Module__gatePrms__C;

 
typedef xdc_runtime_Types_ModuleId CT__xdc_runtime_Memory_Module__id;
extern  const CT__xdc_runtime_Memory_Module__id xdc_runtime_Memory_Module__id__C;

 
typedef xdc_Bool CT__xdc_runtime_Memory_Module__loggerDefined;
extern  const CT__xdc_runtime_Memory_Module__loggerDefined xdc_runtime_Memory_Module__loggerDefined__C;

 
typedef xdc_Ptr CT__xdc_runtime_Memory_Module__loggerObj;
extern  const CT__xdc_runtime_Memory_Module__loggerObj xdc_runtime_Memory_Module__loggerObj__C;

 
typedef xdc_runtime_Types_LoggerFxn0 CT__xdc_runtime_Memory_Module__loggerFxn0;
extern  const CT__xdc_runtime_Memory_Module__loggerFxn0 xdc_runtime_Memory_Module__loggerFxn0__C;

 
typedef xdc_runtime_Types_LoggerFxn1 CT__xdc_runtime_Memory_Module__loggerFxn1;
extern  const CT__xdc_runtime_Memory_Module__loggerFxn1 xdc_runtime_Memory_Module__loggerFxn1__C;

 
typedef xdc_runtime_Types_LoggerFxn2 CT__xdc_runtime_Memory_Module__loggerFxn2;
extern  const CT__xdc_runtime_Memory_Module__loggerFxn2 xdc_runtime_Memory_Module__loggerFxn2__C;

 
typedef xdc_runtime_Types_LoggerFxn4 CT__xdc_runtime_Memory_Module__loggerFxn4;
extern  const CT__xdc_runtime_Memory_Module__loggerFxn4 xdc_runtime_Memory_Module__loggerFxn4__C;

 
typedef xdc_runtime_Types_LoggerFxn8 CT__xdc_runtime_Memory_Module__loggerFxn8;
extern  const CT__xdc_runtime_Memory_Module__loggerFxn8 xdc_runtime_Memory_Module__loggerFxn8__C;

 
typedef xdc_Bool (*CT__xdc_runtime_Memory_Module__startupDoneFxn)(void);
extern  const CT__xdc_runtime_Memory_Module__startupDoneFxn xdc_runtime_Memory_Module__startupDoneFxn__C;

 
typedef xdc_Int CT__xdc_runtime_Memory_Object__count;
extern  const CT__xdc_runtime_Memory_Object__count xdc_runtime_Memory_Object__count__C;

 
typedef xdc_runtime_IHeap_Handle CT__xdc_runtime_Memory_Object__heap;
extern  const CT__xdc_runtime_Memory_Object__heap xdc_runtime_Memory_Object__heap__C;

 
typedef xdc_SizeT CT__xdc_runtime_Memory_Object__sizeof;
extern  const CT__xdc_runtime_Memory_Object__sizeof xdc_runtime_Memory_Object__sizeof__C;

 
typedef xdc_Ptr CT__xdc_runtime_Memory_Object__table;
extern  const CT__xdc_runtime_Memory_Object__table xdc_runtime_Memory_Object__table__C;

 

typedef xdc_runtime_IHeap_Handle CT__xdc_runtime_Memory_defaultHeapInstance;
extern  const CT__xdc_runtime_Memory_defaultHeapInstance xdc_runtime_Memory_defaultHeapInstance__C;




 

 


 

extern xdc_Bool xdc_runtime_Memory_Module__startupDone__S( void );

 


extern xdc_Ptr xdc_runtime_Memory_alloc__E( xdc_runtime_IHeap_Handle heap, xdc_SizeT size, xdc_SizeT align, xdc_runtime_Error_Block *eb );

 


extern xdc_Ptr xdc_runtime_Memory_calloc__E( xdc_runtime_IHeap_Handle heap, xdc_SizeT size, xdc_SizeT align, xdc_runtime_Error_Block *eb );

 


extern void xdc_runtime_Memory_free__E( xdc_runtime_IHeap_Handle heap, xdc_Ptr block, xdc_SizeT size );

 


extern void xdc_runtime_Memory_getStats__E( xdc_runtime_IHeap_Handle heap, xdc_runtime_Memory_Stats *stats );

 


extern xdc_Bool xdc_runtime_Memory_query__E( xdc_runtime_IHeap_Handle heap, xdc_Int qual );

 


extern xdc_SizeT xdc_runtime_Memory_getMaxDefaultTypeAlign__E( void );

 


extern xdc_Ptr xdc_runtime_Memory_valloc__E( xdc_runtime_IHeap_Handle heap, xdc_SizeT size, xdc_SizeT align, xdc_Char value, xdc_runtime_Error_Block *eb );




 

 


 


 


 
static inline CT__xdc_runtime_Memory_Module__id xdc_runtime_Memory_Module_id( void ) 
{
    return xdc_runtime_Memory_Module__id__C;
}

 
static inline xdc_Bool xdc_runtime_Memory_Module_hasMask( void ) 
{
    return xdc_runtime_Memory_Module__diagsMask__C != 0;
}

 
static inline xdc_Bits16 xdc_runtime_Memory_Module_getMask( void ) 
{
    return xdc_runtime_Memory_Module__diagsMask__C != 0 ? *xdc_runtime_Memory_Module__diagsMask__C : 0;
}

 
static inline void xdc_runtime_Memory_Module_setMask( xdc_Bits16 mask ) 
{
    if (xdc_runtime_Memory_Module__diagsMask__C != 0) *xdc_runtime_Memory_Module__diagsMask__C = mask;
}




 










 

#line 283 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.h"




 

#line 322 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Memory.h"


 

#line 59 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"




 




 

 
struct xdc_runtime_IHeap_Params {
    size_t __size;
    const void *__self;
    void *__fxns;
    xdc_runtime_IInstance_Params *instance;
};




 

 
struct xdc_runtime_IHeap_Fxns__ {
    xdc_runtime_Types_Base* __base;
    const xdc_runtime_Types_SysFxns2 *__sysp;
    xdc_Ptr (*alloc)(void*, xdc_SizeT, xdc_SizeT, xdc_runtime_Error_Block*);
    void (*free)(void*, xdc_Ptr, xdc_SizeT);
    xdc_Bool (*isBlocking)(void*);
    void (*getStats)(void*, xdc_runtime_Memory_Stats*);
    xdc_runtime_Types_SysFxns2 __sfxns;
};

 
extern const xdc_runtime_Types_Base xdc_runtime_IHeap_Interface__BASE__C;




 

 

extern xdc_runtime_IHeap_Handle xdc_runtime_IHeap_create(xdc_runtime_IHeap_Module, const xdc_runtime_IHeap_Params *, xdc_runtime_Error_Block *__eb);

 

extern void xdc_runtime_IHeap_delete(xdc_runtime_IHeap_Handle *);

 
static inline xdc_runtime_IHeap_Module xdc_runtime_IHeap_Handle_to_Module( xdc_runtime_IHeap_Handle inst )
{
    return inst->__fxns;
}

 
static inline xdc_runtime_Types_Label *xdc_runtime_IHeap_Handle_label( xdc_runtime_IHeap_Handle inst, xdc_runtime_Types_Label *lab )
{
    return inst->__fxns->__sysp->__label(inst, lab);
}

 
static inline xdc_runtime_Types_ModuleId xdc_runtime_IHeap_Module_id( xdc_runtime_IHeap_Module mod )
{
    return mod->__sysp->__mid;
}

 
static inline xdc_Ptr xdc_runtime_IHeap_alloc( xdc_runtime_IHeap_Handle __inst, xdc_SizeT size, xdc_SizeT align, xdc_runtime_Error_Block *eb )
{
    return __inst->__fxns->alloc((void*)__inst, size, align, eb);
}

 
static inline void xdc_runtime_IHeap_free( xdc_runtime_IHeap_Handle __inst, xdc_Ptr block, xdc_SizeT size )
{
    __inst->__fxns->free((void*)__inst, block, size);
}

 
static inline xdc_Bool xdc_runtime_IHeap_isBlocking( xdc_runtime_IHeap_Handle __inst )
{
    return __inst->__fxns->isBlocking((void*)__inst);
}

 
static inline void xdc_runtime_IHeap_getStats( xdc_runtime_IHeap_Handle __inst, xdc_runtime_Memory_Stats *stats )
{
    __inst->__fxns->getStats((void*)__inst, stats);
}




 






 

 
typedef xdc_Ptr (*xdc_runtime_IHeap_alloc_FxnT)(void *, xdc_SizeT, xdc_SizeT, xdc_runtime_Error_Block*);
static inline xdc_runtime_IHeap_alloc_FxnT xdc_runtime_IHeap_alloc_fxnP( xdc_runtime_IHeap_Handle __inst )
{
    return (xdc_runtime_IHeap_alloc_FxnT)__inst->__fxns->alloc;
}

 
typedef void (*xdc_runtime_IHeap_free_FxnT)(void *, xdc_Ptr, xdc_SizeT);
static inline xdc_runtime_IHeap_free_FxnT xdc_runtime_IHeap_free_fxnP( xdc_runtime_IHeap_Handle __inst )
{
    return (xdc_runtime_IHeap_free_FxnT)__inst->__fxns->free;
}

 
typedef xdc_Bool (*xdc_runtime_IHeap_isBlocking_FxnT)(void *);
static inline xdc_runtime_IHeap_isBlocking_FxnT xdc_runtime_IHeap_isBlocking_fxnP( xdc_runtime_IHeap_Handle __inst )
{
    return (xdc_runtime_IHeap_isBlocking_FxnT)__inst->__fxns->isBlocking;
}

 
typedef void (*xdc_runtime_IHeap_getStats_FxnT)(void *, xdc_runtime_Memory_Stats*);
static inline xdc_runtime_IHeap_getStats_FxnT xdc_runtime_IHeap_getStats_fxnP( xdc_runtime_IHeap_Handle __inst )
{
    return (xdc_runtime_IHeap_getStats_FxnT)__inst->__fxns->getStats;
}




 










 

#line 238 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IHeap.h"


 

#line 56 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"
#line 1 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"





 















 




 

#line 347 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"




 

#line 369 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"




 

#line 425 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/Error.h"


 

#line 57 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"




 




 

 
struct xdc_runtime_IModule_Fxns__ {
    xdc_runtime_Types_Base* __base;
    const xdc_runtime_Types_SysFxns2 *__sysp;
    xdc_runtime_Types_SysFxns2 __sfxns;
};

 
extern const xdc_runtime_Types_Base xdc_runtime_IModule_Interface__BASE__C;




 

 
static inline xdc_runtime_Types_ModuleId xdc_runtime_IModule_Module_id( xdc_runtime_IModule_Module mod )
{
    return mod->__sysp->__mid;
}




 




 










 

#line 121 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/IModule.h"


 

#line 55 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/ISystemSupport.h"




 




 

 
struct xdc_runtime_ISystemSupport_Fxns__ {
    xdc_runtime_Types_Base* __base;
    const xdc_runtime_Types_SysFxns2 *__sysp;
    void (*abort)(xdc_CString);
    void (*exit)(xdc_Int);
    void (*flush)(void);
    void (*putch)(xdc_Char);
    xdc_Bool (*ready)(void);
    xdc_runtime_Types_SysFxns2 __sfxns;
};

 
extern const xdc_runtime_Types_Base xdc_runtime_ISystemSupport_Interface__BASE__C;




 

 
static inline xdc_runtime_Types_ModuleId xdc_runtime_ISystemSupport_Module_id( xdc_runtime_ISystemSupport_Module mod )
{
    return mod->__sysp->__mid;
}

 
static inline void xdc_runtime_ISystemSupport_abort( xdc_runtime_ISystemSupport_Module __inst, xdc_CString str )
{
    __inst->abort(str);
}

 
static inline void xdc_runtime_ISystemSupport_exit( xdc_runtime_ISystemSupport_Module __inst, xdc_Int stat )
{
    __inst->exit(stat);
}

 
static inline void xdc_runtime_ISystemSupport_flush( xdc_runtime_ISystemSupport_Module __inst )
{
    __inst->flush();
}

 
static inline void xdc_runtime_ISystemSupport_putch( xdc_runtime_ISystemSupport_Module __inst, xdc_Char ch )
{
    __inst->putch(ch);
}

 
static inline xdc_Bool xdc_runtime_ISystemSupport_ready( xdc_runtime_ISystemSupport_Module __inst )
{
    return __inst->ready();
}




 






 

 
typedef void (*xdc_runtime_ISystemSupport_abort_FxnT)(xdc_CString);
static inline xdc_runtime_ISystemSupport_abort_FxnT xdc_runtime_ISystemSupport_abort_fxnP( xdc_runtime_ISystemSupport_Module __inst )
{
    return (xdc_runtime_ISystemSupport_abort_FxnT)__inst->abort;
}

 
typedef void (*xdc_runtime_ISystemSupport_exit_FxnT)(xdc_Int);
static inline xdc_runtime_ISystemSupport_exit_FxnT xdc_runtime_ISystemSupport_exit_fxnP( xdc_runtime_ISystemSupport_Module __inst )
{
    return (xdc_runtime_ISystemSupport_exit_FxnT)__inst->exit;
}

 
typedef void (*xdc_runtime_ISystemSupport_flush_FxnT)(void);
static inline xdc_runtime_ISystemSupport_flush_FxnT xdc_runtime_ISystemSupport_flush_fxnP( xdc_runtime_ISystemSupport_Module __inst )
{
    return (xdc_runtime_ISystemSupport_flush_FxnT)__inst->flush;
}

 
typedef void (*xdc_runtime_ISystemSupport_putch_FxnT)(xdc_Char);
static inline xdc_runtime_ISystemSupport_putch_FxnT xdc_runtime_ISystemSupport_putch_fxnP( xdc_runtime_ISystemSupport_Module __inst )
{
    return (xdc_runtime_ISystemSupport_putch_FxnT)__inst->putch;
}

 
typedef xdc_Bool (*xdc_runtime_ISystemSupport_ready_FxnT)(void);
static inline xdc_runtime_ISystemSupport_ready_FxnT xdc_runtime_ISystemSupport_ready_fxnP( xdc_runtime_ISystemSupport_Module __inst )
{
    return (xdc_runtime_ISystemSupport_ready_FxnT)__inst->ready;
}




 










 

#line 211 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/ISystemSupport.h"


 

#line 59 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.h"




 

 
typedef void (*xdc_runtime_SysCallback_AbortFxn)(xdc_CString);

 
typedef void (*xdc_runtime_SysCallback_ExitFxn)(xdc_Int);

 
typedef void (*xdc_runtime_SysCallback_FlushFxn)(void);

 
typedef void (*xdc_runtime_SysCallback_PutchFxn)(xdc_Char);

 
typedef xdc_Bool (*xdc_runtime_SysCallback_ReadyFxn)(void);




 




 

 
typedef xdc_Bits32 CT__xdc_runtime_SysCallback_Module__diagsEnabled;
extern  const CT__xdc_runtime_SysCallback_Module__diagsEnabled xdc_runtime_SysCallback_Module__diagsEnabled__C;

 
typedef xdc_Bits32 CT__xdc_runtime_SysCallback_Module__diagsIncluded;
extern  const CT__xdc_runtime_SysCallback_Module__diagsIncluded xdc_runtime_SysCallback_Module__diagsIncluded__C;

 
typedef xdc_Bits16 *CT__xdc_runtime_SysCallback_Module__diagsMask;
extern  const CT__xdc_runtime_SysCallback_Module__diagsMask xdc_runtime_SysCallback_Module__diagsMask__C;

 
typedef xdc_Ptr CT__xdc_runtime_SysCallback_Module__gateObj;
extern  const CT__xdc_runtime_SysCallback_Module__gateObj xdc_runtime_SysCallback_Module__gateObj__C;

 
typedef xdc_Ptr CT__xdc_runtime_SysCallback_Module__gatePrms;
extern  const CT__xdc_runtime_SysCallback_Module__gatePrms xdc_runtime_SysCallback_Module__gatePrms__C;

 
typedef xdc_runtime_Types_ModuleId CT__xdc_runtime_SysCallback_Module__id;
extern  const CT__xdc_runtime_SysCallback_Module__id xdc_runtime_SysCallback_Module__id__C;

 
typedef xdc_Bool CT__xdc_runtime_SysCallback_Module__loggerDefined;
extern  const CT__xdc_runtime_SysCallback_Module__loggerDefined xdc_runtime_SysCallback_Module__loggerDefined__C;

 
typedef xdc_Ptr CT__xdc_runtime_SysCallback_Module__loggerObj;
extern  const CT__xdc_runtime_SysCallback_Module__loggerObj xdc_runtime_SysCallback_Module__loggerObj__C;

 
typedef xdc_runtime_Types_LoggerFxn0 CT__xdc_runtime_SysCallback_Module__loggerFxn0;
extern  const CT__xdc_runtime_SysCallback_Module__loggerFxn0 xdc_runtime_SysCallback_Module__loggerFxn0__C;

 
typedef xdc_runtime_Types_LoggerFxn1 CT__xdc_runtime_SysCallback_Module__loggerFxn1;
extern  const CT__xdc_runtime_SysCallback_Module__loggerFxn1 xdc_runtime_SysCallback_Module__loggerFxn1__C;

 
typedef xdc_runtime_Types_LoggerFxn2 CT__xdc_runtime_SysCallback_Module__loggerFxn2;
extern  const CT__xdc_runtime_SysCallback_Module__loggerFxn2 xdc_runtime_SysCallback_Module__loggerFxn2__C;

 
typedef xdc_runtime_Types_LoggerFxn4 CT__xdc_runtime_SysCallback_Module__loggerFxn4;
extern  const CT__xdc_runtime_SysCallback_Module__loggerFxn4 xdc_runtime_SysCallback_Module__loggerFxn4__C;

 
typedef xdc_runtime_Types_LoggerFxn8 CT__xdc_runtime_SysCallback_Module__loggerFxn8;
extern  const CT__xdc_runtime_SysCallback_Module__loggerFxn8 xdc_runtime_SysCallback_Module__loggerFxn8__C;

 
typedef xdc_Bool (*CT__xdc_runtime_SysCallback_Module__startupDoneFxn)(void);
extern  const CT__xdc_runtime_SysCallback_Module__startupDoneFxn xdc_runtime_SysCallback_Module__startupDoneFxn__C;

 
typedef xdc_Int CT__xdc_runtime_SysCallback_Object__count;
extern  const CT__xdc_runtime_SysCallback_Object__count xdc_runtime_SysCallback_Object__count__C;

 
typedef xdc_runtime_IHeap_Handle CT__xdc_runtime_SysCallback_Object__heap;
extern  const CT__xdc_runtime_SysCallback_Object__heap xdc_runtime_SysCallback_Object__heap__C;

 
typedef xdc_SizeT CT__xdc_runtime_SysCallback_Object__sizeof;
extern  const CT__xdc_runtime_SysCallback_Object__sizeof xdc_runtime_SysCallback_Object__sizeof__C;

 
typedef xdc_Ptr CT__xdc_runtime_SysCallback_Object__table;
extern  const CT__xdc_runtime_SysCallback_Object__table xdc_runtime_SysCallback_Object__table__C;

 

typedef xdc_runtime_SysCallback_AbortFxn CT__xdc_runtime_SysCallback_abortFxn;
extern  const CT__xdc_runtime_SysCallback_abortFxn xdc_runtime_SysCallback_abortFxn__C;

 

typedef xdc_runtime_SysCallback_ExitFxn CT__xdc_runtime_SysCallback_exitFxn;
extern  const CT__xdc_runtime_SysCallback_exitFxn xdc_runtime_SysCallback_exitFxn__C;

 

typedef xdc_runtime_SysCallback_FlushFxn CT__xdc_runtime_SysCallback_flushFxn;
extern  const CT__xdc_runtime_SysCallback_flushFxn xdc_runtime_SysCallback_flushFxn__C;

 

typedef xdc_runtime_SysCallback_PutchFxn CT__xdc_runtime_SysCallback_putchFxn;
extern  const CT__xdc_runtime_SysCallback_putchFxn xdc_runtime_SysCallback_putchFxn__C;

 

typedef xdc_runtime_SysCallback_ReadyFxn CT__xdc_runtime_SysCallback_readyFxn;
extern  const CT__xdc_runtime_SysCallback_readyFxn xdc_runtime_SysCallback_readyFxn__C;




 

 
struct xdc_runtime_SysCallback_Fxns__ {
    xdc_runtime_Types_Base* __base;
    const xdc_runtime_Types_SysFxns2 *__sysp;
    void (*abort)(xdc_CString);
    void (*exit)(xdc_Int);
    void (*flush)(void);
    void (*putch)(xdc_Char);
    xdc_Bool (*ready)(void);
    xdc_runtime_Types_SysFxns2 __sfxns;
};

 
extern const xdc_runtime_SysCallback_Fxns__ xdc_runtime_SysCallback_Module__FXNS__C;




 

 


 

extern xdc_Bool xdc_runtime_SysCallback_Module__startupDone__S( void );

 


extern void xdc_runtime_SysCallback_abort__E( xdc_CString str );

 


extern void xdc_runtime_SysCallback_exit__E( xdc_Int stat );

 


extern void xdc_runtime_SysCallback_flush__E( void );

 


extern void xdc_runtime_SysCallback_putch__E( xdc_Char ch );

 


extern xdc_Bool xdc_runtime_SysCallback_ready__E( void );




 

 
static inline xdc_runtime_ISystemSupport_Module xdc_runtime_SysCallback_Module_upCast( void )
{
    return (xdc_runtime_ISystemSupport_Module)&xdc_runtime_SysCallback_Module__FXNS__C;
}

 





 

 


 


 


 
static inline CT__xdc_runtime_SysCallback_Module__id xdc_runtime_SysCallback_Module_id( void ) 
{
    return xdc_runtime_SysCallback_Module__id__C;
}

 
static inline xdc_Bool xdc_runtime_SysCallback_Module_hasMask( void ) 
{
    return xdc_runtime_SysCallback_Module__diagsMask__C != 0;
}

 
static inline xdc_Bits16 xdc_runtime_SysCallback_Module_getMask( void ) 
{
    return xdc_runtime_SysCallback_Module__diagsMask__C != 0 ? *xdc_runtime_SysCallback_Module__diagsMask__C : 0;
}

 
static inline void xdc_runtime_SysCallback_Module_setMask( xdc_Bits16 mask ) 
{
    if (xdc_runtime_SysCallback_Module__diagsMask__C != 0) *xdc_runtime_SysCallback_Module__diagsMask__C = mask;
}




 










 













 






 
#line 358 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.h"





 

#line 16 "C:\\ti\\xdctools_3_32_00_06_core\\packages\\xdc\\runtime\\package/internal/SysCallback.xdc.h"

#line 23 "C:\\ti\\xdctools_3_32_00_06_core\\packages\\xdc\\runtime\\package/internal/SysCallback.xdc.h"

 



 



 



 



 



 



 



 



 
#line 93 "C:\\ti\\xdctools_3_32_00_06_core\\packages\\xdc\\runtime\\package/internal/SysCallback.xdc.h"


#line 19 "C:/ti/xdctools_3_32_00_06_core/packages/xdc/runtime/SysCallback.c"



 
void xdc_runtime_SysCallback_abort__E(CString str)
{
    (xdc_runtime_SysCallback_abortFxn__C)(str);
}



 
void xdc_runtime_SysCallback_exit__E(Int stat)
{
    (xdc_runtime_SysCallback_exitFxn__C)(stat);
}



 
void xdc_runtime_SysCallback_putch__E(Char ch)
{
    (xdc_runtime_SysCallback_putchFxn__C)(ch);
}



 
void xdc_runtime_SysCallback_flush__E(void)
{
    (xdc_runtime_SysCallback_flushFxn__C)();
}



 
Bool xdc_runtime_SysCallback_ready__E(void)
{
    return ((xdc_runtime_SysCallback_readyFxn__C)());
}




 
Bool xdc_runtime_SysCallback_defaultReady(void)
{
    return (1);
}




 
void xdc_runtime_SysCallback_defaultFlush(void)
{
}




 
void xdc_runtime_SysCallback_defaultAbort(CString str)
{
    for (;;) {
         
    }
}




 
void xdc_runtime_SysCallback_defaultExit(Int stat)
{
}




 
void xdc_runtime_SysCallback_defaultPutch(Char c)
{
}


 

