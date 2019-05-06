// #define LOOPN(n,a) LOOP##n(a)
// #define LOOPF ,
// #define LOOP2(a) a LOOPF a LOOPF
// #define LOOP3(a) a LOOPF a LOOPF a LOOPF
// #define LOOP4(a) a LOOPF a LOOPF a LOOPF a LOOPF
// #define LOOP5(a) a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF
// #define LOOP6(a) a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF
// #define LOOP7(a) a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF
// #define LOOP8(a) a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF
// #define LOOP9(a) a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF a LOOPF


// #define LC_ERRORS_NAMES \
//     Cn(LC_RESPONSE_PLUGIN_OK, -10) \
//     Cw(8) \
//     Cn(LC_RESPONSE_GENERIC_ERROR, -1) \
//     Cn(LC_FT_OK, 0) \
//     Ci(LC_FT_INVALID_HANDLE) \
//     Ci(LC_FT_DEVICE_NOT_FOUND) \
//     Ci(LC_FT_DEVICE_NOT_OPENED) \
//     Ci(LC_FT_IO_ERROR) \
//     Ci(LC_FT_INSUFFICIENT_RESOURCES) \
//     Ci(LC_FT_INVALID_PARAMETER) \
//     Ci(LC_FT_INVALID_BAUD_RATE) \
//     Ci(LC_FT_DEVICE_NOT_OPENED_FOR_ERASE) \
//     Ci(LC_FT_DEVICE_NOT_OPENED_FOR_WRITE) \
//     Ci(LC_FT_FAILED_TO_WRITE_DEVICE) \
//     Ci(LC_FT_EEPROM_READ_FAILED) \
//     Ci(LC_FT_EEPROM_WRITE_FAILED) \
//     Ci(LC_FT_EEPROM_ERASE_FAILED) \
//     Ci(LC_FT_EEPROM_NOT_PRESENT) \
//     Ci(LC_FT_EEPROM_NOT_PROGRAMMED) \
//     Ci(LC_FT_INVALID_ARGS) \
//     Ci(LC_FT_NOT_SUPPORTED) \
//     Ci(LC_FT_OTHER_ERROR) \
//     Ci(LC_FT_DEVICE_LIST_NOT_READY)


// #define Cn(x,y) x=y,
// #define Ci(x) x,
// #define Cw(x)
// enum LC_errors { LC_ERRORS_NAMES TOP };
// #undef Cn
// #undef Ci
// #undef Cw
// #define Cn(x,y) #x,
// #define Ci(x) #x,
// #define Cw(x) LOOPN(x,"")
// static const char* __LC_errors__strings[] = { LC_ERRORS_NAMES };
// static const char** LC_errors__strings = &__LC_errors__strings[10];