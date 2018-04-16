
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sirfdump.h"
#include "sirf_msg.h"
#include "sirf_codec.h"
#include "sirf_codec_nmea.h"
#include "sirf_codec_ssb.h"
#include "sirf_proto_nmea.h"

void GGAtoCSV(char* str, FILE* out_f)
{
    char buf[1024] = "41,";
    char *sep = strtok(str, ",");

    double lat, lon;
    char ns, we;
    if(strncmp(sep, "$GPGGA", 6) == 0)
    {
        int i = 0;
        while(sep != NULL)
        {
            sep = strtok(NULL, ",");
            i++;

            switch(i)
            {
                case 2: //lat
                    lat = atof(sep) / 100;
                    break;
                case 3:
                    ns = sep[0];
                    break;
                case 4:
                    lon = atof(sep) / 100;
                    break;
                case 5:
                    we = sep[0];
                    break;
                default:
                    break;
            }
        }

        if(ns == 'S')
            lat = -lat;
        if(we == 'W')
            lon = -lon;
        
        fprintf(out_f, "41,%.8f,%.8f\n", lat, lon);
    }
    return;     
}

void MSG2toCSV(uint8_t* payload, unsigned int len, FILE* out_f)
{
    /*
    for(int i = 0; i < len; i++)
        printf("%02x ", payload[i]);
    */
    int64_t pos_x, pos_y, pos_z;
    pos_x = (payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | payload[4];
    pos_y = (payload[5] << 24) | (payload[6] << 16) | (payload[7] << 8) | payload[8];
    pos_z = (payload[9] << 24) | (payload[10] << 16) | (payload[11] << 8) | payload[12];

    //printf("%d, %d, %d\n", pos_x, pos_y, pos_z);
    fprintf(out_f, "2,%d,%d,%d\n", pos_x, pos_y, pos_z);
}

int output_csv(struct transport_msg_t *msg, FILE *out_f, void *ctx)
{
   int err;
   tSIRF_UINT32 msg_id, msg_length;
   tSIRF_UINT32 options;
   tSIRF_UINT32 str_size;
   unsigned msg_n;
   unsigned max_msg;
   uint8_t msg_structure[SIRF_MSG_SSB_MAX_MESSAGE_LEN];
   char str[1024];
   int *msg_id_list = (int *)ctx;
   int count = 0;
   int i;

   if (!msg || msg->payload_length < 1)
      return 1;

   options = 0;
   err = SIRF_CODEC_SSB_Decode(msg->payload,
	 msg->payload_length,
	 &msg_id,
	 msg_structure,
	 &msg_length,
         &options);

   if (err)
      return err;

    tSIRF_UINT32 msg_41 = (((SIRF_LC_SSB) & 0xFF) << 16) | (( (0) & 0xFF) << 8) |  ((41) & 0xFF);
    tSIRF_UINT32 msg_2 = (((SIRF_LC_SSB) & 0xFF) << 16) | (( (0) & 0xFF) << 8) |  ((2) & 0xFF);

    if(msg_id == msg_2)
    {
        MSG2toCSV(msg->payload, msg->payload_length, out_f);
        return 0;
    }

   str[0]='\0';
   str_size = sizeof(str);

   if (msg_id == SIRF_MSG_SSB_EE_SEA_PROVIDE_EPH) {
       msg_n = SIRF_CODEC_NMEA_PSRF108;
   }else {
       msg_n = SIRF_CODEC_OPTIONS_GET_FIRST_MSG;
   }
   max_msg = msg_n + 1;
   do {
       options = msg_n;
       if (SIRF_PROTO_NMEA_Encode(msg_id,
                   msg_structure,
                   msg_length,
                   (tSIRF_UINT8 *)str,
                   &str_size,
                   &options) != SIRF_SUCCESS) {
           break;
       }
       msg_n = SIRF_CODEC_OPTIONS_GET_MSG_NUMBER(options) + 1;
       max_msg = SIRF_CODEC_OPTIONS_GET_MAX_MSG_NUMBER(options);
       
       if(msg_id == msg_41)
       {
           GGAtoCSV(str, out_f);
       }
   } while (msg_n < max_msg);

   return err;
}


