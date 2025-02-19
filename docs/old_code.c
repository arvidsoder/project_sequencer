ISR(TIMER0_OVF_vect){ //PAC18
    int8_t diff = Read_encoder() - rotary_enc;
    int8_t increment = 0;

    if (((diff > 0) && (diff < 6)) || (diff < -7)) 
    {
        increment = 1;
    }  else if (((diff < 0) && (diff > -6)) || (diff > 7))
    {
        increment = -1;
    }
    foobar += increment;
    rotary_enc = Read_encoder();
        
}

uint8_t Graytobin[16] = {0,15,7,8,3,12,4,11,1,14,6,9,2,13,5,10}; //pac18r rotary encoder

uint8_t Read_encoder(void){
    uint8_t inv = 0;
    uint8_t gray = 1*((PIND & (1<<ENC_P4))==0) + 2*((PIND & (1<<ENC_P3))==0) + 4*((PINB & (1<<ENC_P2))==0) + 8*((PINB & (1<<ENC_P1))==0);
    inv = Graytobin[gray];
    return inv;
}
