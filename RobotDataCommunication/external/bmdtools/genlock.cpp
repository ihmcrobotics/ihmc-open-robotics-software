/*
 * Copyright (C) 2010-2011 Antonio José Tomás Martínez
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 */

#include "DeckLinkAPI.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int usage(int status);
int kbhit(void);

int	main (int argc, char** argv)
{
    IDeckLinkIterator       *deckLinkIterator;
    IDeckLink               *deckLink;
    IDeckLinkConfiguration  *deckLinkConfiguration;
    IDeckLinkOutput         *deckLinkOutput;
    BMDReferenceStatus      referenceStatus;
    HRESULT                 result;
    int                     offset = 0, camera = 0, i=0;
    int                     ch;
    int                     exitStatus = 1;
    bool                    interactive = false;
    char                    key;

    // Create an IDeckLinkIterator object to enumerate all DeckLink cards in the system
    deckLinkIterator = CreateDeckLinkIteratorInstance();
    if (deckLinkIterator == NULL)
    {
		fprintf(stderr, "#NO DRIVER\n");
		return 1;
    }

    // Parse command line options
    if (argc < 2) usage(0);
    while ((ch = getopt(argc, argv, "?hC:IO:")) != -1)
    {
        switch (ch)
        {
            case 'O':
            	offset = atoi(optarg);
                break;
            case 'C':
            	camera = atoi(optarg);
            	break;
            case 'I':
            	interactive = true;
            	break;
            case '?':
            case 'h':
                usage(0);
                break;
        }
    }

    /* Connect to the first DeckLink instance */
    do {
    	result = deckLinkIterator->Next(&deckLink);
    } while(i++<camera);

    if (result != S_OK)
    {
        fprintf(stderr, "#NO CARD\n");
        goto bail;
    }

    if (deckLink->QueryInterface(IID_IDeckLinkOutput, (void**)&deckLinkOutput) != S_OK)
        goto bail;

    result = deckLink->QueryInterface(IID_IDeckLinkConfiguration, (void**)&deckLinkConfiguration);
    if (result != S_OK)
    {
        fprintf(stderr, "#ERROR: Could not obtain the IDeckLinkConfiguration interface - result = %08x\n", result);
        goto bail;
    }

    // Here is the code for the interactive mode (it assumes REF IN locked)
    if (interactive){
        result=deckLinkOutput->GetReferenceStatus(&referenceStatus);
        if (referenceStatus & bmdReferenceNotSupportedByHardware){
        	fprintf(stderr, "#NO REF IN Input on card = %d\n", camera);
        	goto bail;
        }
    	if(referenceStatus & bmdReferenceLocked){
    		// Now we can start to adjust the time offset to the reference source
    		while(key != 'q'){
    			if (offset <= -511) offset = -511;
    			if (offset >= 511) offset = 511;
    			result=deckLinkConfiguration->SetInt(bmdDeckLinkConfigReferenceInputTimingOffset, (int64_t) offset);
    			if (result != S_OK) fprintf(stderr, "Card:%d Offset:%d ERROR          \r",camera,offset);
    			else                fprintf(stderr, "Card:%d Offset:%d OK             \r",camera,offset);
    			while(!kbhit());key=getchar();
    			if(key == '+') offset++;
    			else if(key == '-') offset--;
    		}
    		goto bail;
    	}else{
        	fprintf(stderr, "#REF IN Input UNLOCKED SOURCE on card = %d\n", camera);
        	goto bail;
    	}
    }

    // Here is the code to make the stuff
    result=deckLinkOutput->GetReferenceStatus(&referenceStatus);
    if (referenceStatus & bmdReferenceNotSupportedByHardware){
    	fprintf(stderr, "#NO REF IN Input on card = %d\n", camera);
    	goto bail;
    }
    else{
    	if(referenceStatus & bmdReferenceLocked) fprintf(stderr, "LOCKED ");
    	else fprintf(stderr, "UNLOCKED ");
        if ((offset >= -511) && (offset <= 511)){
        	result=deckLinkConfiguration->SetInt(bmdDeckLinkConfigReferenceInputTimingOffset, (int64_t) offset);
        	if (result != S_OK) fprintf(stderr, "#OFFSETERROR\n");
        	else{
        		fprintf(stderr, "%d\n",offset);
        		exitStatus = 0;
        	}
        }
        else{
        	fprintf(stderr, "#OUT OF RANGE: Timing offset value is out of range: from -511 to 511\n");
        	goto bail;
        }
    }

bail:
    if (deckLink != NULL)
    {
        deckLink->Release();
        deckLink = NULL;
    }

    if (deckLinkIterator != NULL) deckLinkIterator->Release();

    return exitStatus;

}

int usage(int status)
{
    fprintf(stderr,
        "Usage: genlock [OPTIONS]\n"
        "\n"
        "    -C <num>                 number of card to be used (default = 0)\n"
    	"    -O <ref_in_time_offset>  reference input time offset (default = 0) (min = -511 ; max = 511)\n"
       	"    -I                       interactive mode (press keys: + = increase offset, - = decrease offset, q = exit once fixed\n"
        "\n"
        "Stablish the reference input timing offset eg:\n"
        "\n"
        "    genlock -C 0 -O 5\n\n\n"
    );

    exit(status);
}

int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
		ungetc(ch, stdin);
		return 1;
    }

    return 0;
}

