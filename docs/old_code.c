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