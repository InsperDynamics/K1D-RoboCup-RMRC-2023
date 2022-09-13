#include <Encoder.h>
Encoder EB(29, 27);
Encoder DB(25, 23);
Encoder EA(30, 32);
Encoder DA(44, 42);
//E=left , D=right
//A=front , B=back
int EncPulseEB=0;
int EncPulseDB=0;
int EncPulseEA=0;
int EncPulseDA=0;
int EncPulseAvg=0;

void UpdateEncoders(){
    EncPulseEB=abs(EB.read());
    EncPulseEA=abs(EA.read());
    EncPulseDB=abs(DB.read());
    EncPulseDA=abs(DA.read());
    EncPulseAvg=(EncPulseEB+EncPulseEA+EncPulseDA+EncPulseDB)/4;
}

void ResetEncoders(){
    EB.write(0);
    EA.write(0);
    DB.write(0);
    DA.write(0);
    EncPulseEB=0;
    EncPulseEA=0;
    EncPulseDB=0;
    EncPulseDA=0;
    EncPulseAvg=0;
}