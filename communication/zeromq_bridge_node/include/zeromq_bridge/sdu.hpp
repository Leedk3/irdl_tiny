#include <stdint.h>

#define MAX_SDU_MSG_LEN		     1024

#define MSG_ID_PL_00		     0
#define MSG_ID_PL_01_00		     1
#define MSG_ID_PL_01_01		     2

#pragma pack(push, 1)		     // 현재 정렬 상태를 스택에 저장하고 1바이트 정렬 지정

typedef struct {
    uint16_t id;
    uint16_t len;
    int8_t   sdu[MAX_SDU_MSG_LEN];
} SDU_CONTAINER_T;

typedef struct {                     //: offset bits
    uint8_t  payload_type_code;      //:    0    3  
    uint8_t  address_qualifier;      //:    3    3  
    uint32_t icao;                   //:    6   24  
    uint8_t  security_mode;          //:   30    2  
    double   latitude_wgs84;         //:   32   26  
    double   longitude_wgs84;        //:   58   27  
    double   true_altitude_ft;       //:   85   12  
    double   pressure_altitude_ft;   //:   97   12  
    double   absolute_altitude_ft;   //:  109   12  
    uint8_t  ags;                    //:  121    1  
    uint8_t  vst;                    //:  122    1  
    uint8_t  hvlc_ns_dir;            //:  123    1  
    double   hvlc_ns_velocity;       //:  124   10  
    uint8_t  hvlc_ew_dir;            //:  134    1  
    double   hvlc_ew_velocity;       //:  135   10  
    uint8_t  vvlc_source;            //:  145    1  
    uint8_t  vvlc_up_down;           //:  146    1  
    double   vvlc_velocity;          //:  147    9  
    uint8_t  utc_coupled;            //:  156    1  
    uint8_t  nic;                    //:  157    4  
    uint8_t  nacp;                   //:  161    4  
    uint8_t  nacv;                   //:  165    3  
    uint8_t  gva;                    //:  168    2  
    uint8_t  sil;                    //:  170    2  
    uint8_t  sda;                    //:  172    2  
    uint8_t  talt_head_valid;        //:  174    1  
    uint8_t  talt_type;              //:  175    1  
    double   talt_height;            //:  176   12  
    uint16_t talt_pressure_value;    //:  188    9  
    uint8_t  head_valid;             //:  197    1  
    uint8_t  head_dir_sign;          //:  198    1  
    double   head_dir_value;         //:  199    8  
    uint8_t  fmc_st;                 //:  207    1  
    uint8_t  fmc_ap;                 //:  208    1  
    uint8_t  fmc_vnav;               //:  209    1  
    uint8_t  fmc_alt;                //:  210    1  
    uint8_t  fmc_app;                //:  211    1  
    uint8_t  fmc_lnav;               //:  212    1  
    uint32_t reserved_bits;          //:  213   27  
    uint32_t sign;                   //:  240   32  
} V2VB_PL0_MSG_T;

#pragma pack(pop)		     // 정렬 상태를 원래대로 복원
