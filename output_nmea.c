
#include <stdio.h>
#include "sirfdump.h"
#include "sirf_msg.h"
#include "sirf_codec.h"
#include "sirf_codec_nmea.h"
#include "sirf_codec_ssb.h"
#include "sirf_proto_nmea.h"

int output_nmea(struct transport_msg_t *msg, FILE *out_f, void *ctx)
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

   if (msg_id_list != NULL)
   {
       count = msg_id_list[0];
       for(i = 1; i < count +1; i++)
       {
           tSIRF_UINT32 msg_32 = (((SIRF_LC_SSB) & 0xFF) << 16) | (( (0) & 0xFF) << 8) |  ((msg_id_list[i]) & 0xFF);

           if(msg_id == msg_32)
                break;
       }

       if(i == count+1)
       {
           //printf("msg %d not processed\n", msg_id);
            return 1;
       }
   }

    /*
   if(msg_id_list[i] == 2)
    printf("msg2");
    */

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
       //mif(ctx == NULL)
        fputs(str, out_f);
       //else
       //{
        //printf("%d: %s", msg_id_list[i], str);
       //}
   } while (msg_n < max_msg);

   return err;
}


