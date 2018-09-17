/* IRLib_P13_Bryant.h
 * Part of IRLib Library for Arduino receiving, decoding, and sending
 * infrared signals. See COPYRIGHT.txt and LICENSE.txt for more information.
 */

/*
 * Bryant Protocol
 * Ductless heat pump with remote control model RG52F3/BGEFU1
 * It consists of zzzzzzzzz
 */
#ifndef IRLIB_PROTOCOL_13_H
#define IRLIB_PROTOCOL_13_H
#define IR_SEND_PROTOCOL_13		case 13: IRsendBryant::send(data); break;
#define IR_DECODE_PROTOCOL_13	if(IRdecodeBryant::decode()) return true;
#ifdef IRLIB_HAVE_COMBO
	#define PV_IR_DECODE_PROTOCOL_13 ,public virtual IRdecodeBryant
	#define PV_IR_SEND_PROTOCOL_13   ,public virtual IRsendBryant
#else
	#define PV_IR_DECODE_PROTOCOL_13  public virtual IRdecodeBryant
	#define PV_IR_SEND_PROTOCOL_13    public virtual IRsendBryant
#endif

#ifdef IRLIBSENDBASE_H
class IRsendBryant: public virtual IRsendBase {
  public:
    void send(uint32_t address) {
      enableIROut(38);
      mark(500*9); space(500*8);	//Send header
      for (int i = 0; i < 6; i++)
        putBits(bytes[i], 8);

      mark(500); space (500*9);	//Send break
      mark(500*9); space(500*8);	//Send header

      for (int i = 0; i < 6; i++)
        putBits(~bytes[i], 8);
      mark(500);               	//Send completion mark
    };
  private:
    /* Because not all of the data bits are contiguous in the stream,
    * we created this little routine to be called multiple times to send a 
    * segment of the data. 
    */
    void putBits (uint8_t data, uint8_t nbits) {
      for (uint8_t i = 0; i < nbits; i++) {
        if (data & 0x80) {
          mark(500);  space(500*3);
        } else {
          mark(500);  space(500);
        };
        data <<= 1;
      }
    }
};

#endif  //IRLIBSENDBASE_H

#ifdef IRLIBDECODEBASE_H
#define BRYANT_DATALEN 12
uint8_t collectedData[BRYANT_DATALEN];
class IRdecodeBryant: public virtual IRdecodeBase {
  public:
    bool decode(void) {
      IRLIB_ATTEMPT_MESSAGE(F("Bryant"));
      if (recvGlobal.decodeLength != 100) return RAW_COUNT_ERROR;
      if (!MATCH(recvGlobal.decodeBuffer[1],500*9)) return HEADER_MARK_ERROR(500*9);
      if (!MATCH(recvGlobal.decodeBuffer[2],500*9)) return HEADER_SPACE_ERROR(500*9);
      protocolNum = BRYANT;
      offset=2; data=0;
      for (int i = 0; i < BRYANT_DATALEN; i++) {
          if (!getBits(offset+(8*2))) return false;
          collectedData[i] = data;
      }
      address = (uint16_t) &collectedData;
      bits = 8 * BRYANT_DATALEN;		//set bit length
      return true;
    };
  private:
    /* Because not all of the data bits are contiguous in the stream
     * we created this little routine to be called multiple times
     * to decode a segment of the data. Parameter "last_offset" is when we
     * stop decoding the segment.
     */
    bool getBits(uint8_t last_offset) {
      while (offset < last_offset) {
        if (!MATCH(recvGlobal.decodeBuffer[offset],500)) return DATA_MARK_ERROR(500);
        offset++;
        if (MATCH(recvGlobal.decodeBuffer[offset],500*3)) 
          data = (data << 1) | 1;
        else if (MATCH(recvGlobal.decodeBuffer[offset],500)) 
          data <<= 1;
        else return DATA_SPACE_ERROR(500*3);
        offset++;
      };
      return true;
    };
    uint8_t offset;
    uint32_t data;
};
#endif //IRLIBDECODEBASE_H

#define IRLIB_HAVE_COMBO

#endif //IRLIB_PROTOCOL_13_H
