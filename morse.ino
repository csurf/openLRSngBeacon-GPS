// simple morse encoder 
// based partly on code found here:
// https://gist.github.com/madc/4474559


// remove unwanted char's to save space
// parser wll simply ignore them
static const struct {const char letter, *code;} MorseMap[] =
{
	{ '1', ".----" },
	{ '2', "..---" },
	{ '3', "...--" },
	{ '4', "....-" },
	{ '5', "....." },
	{ '6', "-...." },
	{ '7', "--..." },
	{ '8', "---.." },
	{ '9', "----." },
	{ '0', "-----" },	
	{ '.', ".-.-.-" },
	{ '-', "-....-" },
	{ ' ', "     " }, //Gap between word, seven units 	
	{ 'A', ".-" },
	{ 'B', "-..." },
	{ 'C', "-.-." },
	{ 'D', "-.." },
	{ 'E', "." },
	{ 'F', "..-." },
	{ 'G', "--." },
	{ 'H', "...." },
	{ 'I', ".." },
	{ 'J', ".---" },
	{ 'K', "-.-" },
	{ 'L', ".-.." },
	{ 'M', "--" },
	{ 'N', "-." },
	{ 'O', "---" },
	{ 'P', ".--." },
	{ 'Q', "--.-" },
	{ 'R', ".-." },
	{ 'S', "..." },
	{ 'T', "-" },
	{ 'U', "..-" },
	{ 'V', "...-" },
	{ 'W', ".--" },
	{ 'X', "-..-" },
	{ 'Y', "-.--" },
	{ 'Z', "--.." },
	
	{ ',', "--..--" },
	{ '?', "..--.." },
	{ '!', "-.-.--" },
	{ ':', "---..." },
	{ ';', "-.-.-." },
	{ '(', "-.--." },
	{ ')', "-.--.-" },
	{ '"', ".-..-." },
	{ '@', ".--.-." },
	{ '&', ".-..." },
};

void morseEncode(const char *string)
{
	size_t i, j, l;
	l = strlen(string);
	
	size_t m =  sizeof MorseMap / sizeof *MorseMap;
	
	for( i = 0; i < l; i++) {
		for( j = 0; j < m; j++ ) {
			if( toupper(string[i]) == MorseMap[j].letter ) {
				
				#ifdef DEBUG_ENCODE
				Serial.print(MorseMap[j].letter);
				Serial.print(" | ");
				Serial.println(MorseMap[j].code);
				#endif
				
				morseSend(MorseMap[j].code);
				delay(MORSE_CHARSPACE_LEN); // mandatory pause after each char
				break;
			}
		}
		
	}
	#ifdef DEBUG_ENCODE
	Serial.println();
	Serial.println();
	#endif
}

void morseSend(const char *code)
{
	uint8_t l = strlen(code); //255 char max input string
	for( uint8_t i = 0 ; i < l; i++ ) // find a better loop?
	{ 
		char c = code[i];
		switch( c)
		{
			case '.': //dot
			beaconTone(MORSE_TONE_HZ, MORSE_DOT_LEN);
			break;

			case '-': //dash
			beaconTone(MORSE_TONE_HZ, MORSE_DASH_LEN);
			break;

			case ' ': //pause falls thru
			break;
		}
		delay(MORSE_DOT_LEN); // mandatory pause after each tone
	}
}
