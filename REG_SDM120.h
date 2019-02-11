#define ID_meter  0x01
#define Total_of_Reg  6

#define Reg_Volt                0x0000      //  0.
#define Reg_Current             0x0006      //  1.
#define Reg_ActivePower         0x000C      //  2.          
#define Reg_PowerFactor         0x001E      //  3.
#define Reg_Frequency           0x0046      //  4.
#define Reg_TotalActiveEnergy   0x0156      //  5.

uint16_t const Reg_addr[6] = {
  Reg_Volt,
  Reg_Current,
  Reg_ActivePower,
  Reg_PowerFactor,
  Reg_Frequency,
  Reg_TotalActiveEnergy
};

float DATA_METER [Total_of_Reg] ;

