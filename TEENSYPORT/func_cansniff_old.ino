// old and busted.  it worked but its ugly
void canSniff(const CAN_message_t &msg) {
    
  if (verbose) {
    Serial.print("[VERBOSE] MB "); Serial.print(msg.mb);
    Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" EXT: "); Serial.print(msg.flags.extended);
    Serial.print(" TS: "); Serial.print(msg.timestamp);
    Serial.print(" ID: "); Serial.print(msg.id, HEX);
    Serial.print(" IDD: "); Serial.print(msg.id);
    Serial.print(" Buffer: ");
    for ( uint8_t i = 0; i < msg.len; i++ ) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
    } Serial.println();
  }


  if (msg.buf[0] == 0x10) {
    
   
    // original packet by packet parsing
    unsigned char data[4] = {msg.buf[6], msg.buf[5], msg.buf[4], msg.buf[3]};
    feedbackKnockFinal = calcFloatFull(data, 1);

    fineKnockData[3] = msg.buf[7];
    set1 = 1;
    if (set1 & set2) {
      fineKnockFinal = calcFloatFull(fineKnockData, 1);
      set1 = 0;
      set2 = 0;
    }
    //
  } // finished with 0x10

  if (msg.buf[0] == 0x21) {
    fineKnockData[2] = msg.buf[1];
    fineKnockData[1] = msg.buf[2];
    fineKnockData[0] = msg.buf[3];
    set2 = 1;
    if (set1 & set2) {
      fineKnockFinal = calcFloatFull(fineKnockData, 1);
      set1 = 0;
      set2 = 0;
    }

    unsigned char data[4] = {msg.buf[7], msg.buf[6], msg.buf[5], msg.buf[4]};
    boostFinal = calcFloatFull(data, 0.01933677);
  }

  if (msg.buf[0] == 0x22) {
    unsigned char data[2] = {msg.buf[2], msg.buf[1]};
    rpmFinal = calcIntFull(data, .25);
    shift = calcShift(rpmFinal);
    coolantFinal = calcTemp(msg.buf[3]);
    damFinal = calcByteToFloat(msg.buf[4], 0.0625);
    intakeTempFinal = calcTemp(msg.buf[5]);
  }
  
  if (msg.buf[0] == 0x30) {

    if (printStats) {
      Serial.print("30 | FB: "); Serial.print(feedbackKnockFinal);
      Serial.print(" FN: "); Serial.print(fineKnockFinal);
      Serial.print(" BST: "); Serial.print(boostFinal);
      Serial.print(" COOL: "); Serial.print(coolantFinal);
      Serial.print(" DAM: "); Serial.print(damFinal);
      Serial.print(" INTAKE: "); Serial.print(intakeTempFinal);
      Serial.print(" OIL T: "); Serial.print(oilTemperature);
      Serial.print(" OIL P: "); Serial.println(oilPressure);
    }

    flowCont = 1;
    for (int i = 1; i < 8; i++) {
      if (msg.buf[i] != 0x00) { flowCont = 0; }
    }
    if (verbose) {
      if (flowCont) { Serial.println("[VERBOSE] **************** FLOW CONTINUE RECEIVED"); }
      else { Serial.println("[VERBOSE] !!!!!!!!!!!!!!!!!!!!!!!!!!! FLOW ERROR RECEIVED"); }
    }
    //cnt++;
  }
}