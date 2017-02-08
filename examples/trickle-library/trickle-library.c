#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include "lib/trickle-timer.h"
#include "dev/leds.h"
#include "lib/random.h"
#include "net/ip/uip.h"

#include <string.h>
#include <stdlib.h> /** MJB04 **/
#include "node-id.h" /** MJB04 **/

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#include "powertrace.h"
/* Trickle variables and constants */
static struct trickle_timer tt;

#define IMIN               16   /* ticks */
#define IMAX               10   /* doublings */
#define REDUNDANCY_CONST    2
#define UIP_IP_BUF  ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
/* Networking */
#define TRICKLE_PROTO_PORT 30001
static struct uip_udp_conn *trickle_conn;
static uip_ipaddr_t ipaddr;     /* destination: link-local all-nodes multicast */

/*
 * For this 'protocol', nodes exchange a token (1 byte) at a frequency
 * governed by trickle. A node detects an inconsistency when it receives a
 * token different than the one it knows.
 * In this case, either:
 * - 'they' have a 'newer' token and we also update our own value, or
 * - 'we' have a 'newer' token, in which case we trigger an inconsistency
 *   without updating our value.
 * In this context, 'newer' is defined in serial number arithmetic terms.
 *
 * Every NEW_TOKEN_INTERVAL clock ticks each node will generate a new token
 * with probability 1/NEW_TOKEN_PROB. This is controlled by etimer et.
 */
#define NEW_TOKEN_INTERVAL  20 * CLOCK_SECOND // sending rate halved for formation simulation
#define NEW_TOKEN_PROB      64 /** MJB04 **/
#define BUFFER_SIZE         32 /** MJB04 **/
static uint16_t token;
static unsigned long time_stamp; /** MJB04 **/   
static char combined[BUFFER_SIZE]; /** MJB04 **/

static struct etimer et; /* Used to periodically generate inconsistencies */
/*---------------------------------------------------------------------------*/
PROCESS(trickle_protocol_process, "Trickle Protocol process");
AUTOSTART_PROCESSES(&trickle_protocol_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  leds_on(LEDS_GREEN);
  /** MJB04 **/
  uint16_t token_h;
  unsigned long ts_h;
  char first_half[10];
  char second_half[6];
  /***/
  if(uip_newdata()) {
    /**************** MJB04 */
    char* mess =(char*)uip_appdata;
    int i = 0;
    int j = 0;
    // debug
    if(!*mess) // a character with all bits set to Zero!
    {
        return;
    }
    //
    while(mess[i] != ' '){
        first_half[i] = mess[i];
        i++;
    }
    
    first_half[i] = '\0';
    ts_h = (unsigned long)atoi(first_half);
    i++;

    while(mess[i] != '\0'){
        second_half[j] = mess[i];
        i++;
        j++;
    }
    second_half[j] = '\0';
    token_h = (uint16_t)atoi(second_half);
    /*****************/


    PRINTF("Received At %lu from  %d ", (unsigned long)clock_time(), UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
    PRINTF("Our token=%d:%lu, theirs=%d:%lu\n", token, time_stamp, token_h, ts_h); /** MJB04 **/

    if(token == token_h && time_stamp == ts_h) {  /** MJB04 **/ 
      PRINTF("Consistent RX\n");
      trickle_timer_consistency(&tt);
    } else {
      if(token < token_h) { /** MJB04 **/ 
        PRINTF("Their token is newer. Update to token %d\n", token_h);
        token = token_h; /** MJB04 **/ 
        time_stamp = ts_h; /** MJB04 **/ 
        sprintf(combined, "%lu %d", time_stamp, token); /** MJB04 **/
      } else if(token == token_h && ts_h < time_stamp) { /** MJB04 **/
        PRINTF("Their timestamp is newer. Update to timestamp %lu\n", ts_h); /** MJB04 **/
        time_stamp = ts_h; /** MJB04 **/
        sprintf(combined, "%lu %d", time_stamp, token); /** MJB04 **/
      } else {
        PRINTF("They are behind\n");
      }
      trickle_timer_inconsistency(&tt);

      /*
       * Here tt.ct.etimer.timer.{start + interval} points to time t in the
       * current interval. However, between t and I it points to the interval's
       * end so if you're going to use this, do so with caution.
       */
      PRINTF("At %lu: Trickle inconsistency. Scheduled TX for %lu\n",
             (unsigned long)clock_time(),
             (unsigned long)(tt.ct.etimer.timer.start +
                             tt.ct.etimer.timer.interval));
    }
  }
  leds_off(LEDS_GREEN);
  return;
}
/*---------------------------------------------------------------------------*/
static void
trickle_tx(void *ptr, uint8_t suppress)
{
  /* *ptr is a pointer to the trickle_timer that triggered this callback. In
   * his example we know that ptr points to tt. However, we pretend that we did
   * not know (which would be the case if we e.g. had multiple trickle timers)
   * and cast it to a local struct trickle_timer* */
  struct trickle_timer *loc_tt = (struct trickle_timer *)ptr;

  if(suppress == TRICKLE_TIMER_TX_SUPPRESS) {
    return;
  }

  leds_on(LEDS_RED);

  /* Instead of changing ->ripaddr around by ourselves, we could have used
   * uip_udp_packet_sendto which would have done it for us. However it puts an
   * extra ~20 bytes on stack and the cc2x3x micros hate it, so we stick with
   * send() */

  /* Destination IP: link-local all-nodes multicast */
  uip_ipaddr_copy(&trickle_conn->ripaddr, &ipaddr);
  uip_udp_packet_send(trickle_conn, &combined, sizeof(combined)); /** MJB04 **/
  PRINTF("Sending %s with a length of %d \n", combined, sizeof(combined)); /** MJB04 **/ 
  /* Restore to 'accept incoming from any IP' */
  uip_create_unspecified(&trickle_conn->ripaddr);

  leds_off(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(trickle_protocol_process, ev, data)
{
  PROCESS_BEGIN();
  powertrace_start(CLOCK_SECOND * 10);
  //powertrace_sniff(POWERTRACE_ON);
  PRINTF("Trickle protocol started in Node: %d\n", node_id);

  uip_create_linklocal_allnodes_mcast(&ipaddr); /* Store for later */

  trickle_conn = udp_new(NULL, UIP_HTONS(TRICKLE_PROTO_PORT), NULL);
  udp_bind(trickle_conn, UIP_HTONS(TRICKLE_PROTO_PORT));

  PRINTF("Connection: local/remote port %u/%u\n",
         UIP_HTONS(trickle_conn->lport), UIP_HTONS(trickle_conn->rport));

  token = 0;

  trickle_timer_config(&tt, IMIN, IMAX, REDUNDANCY_CONST);
  trickle_timer_set(&tt, trickle_tx, &tt);
  /*
   * At this point trickle is started and is running the first interval. All
   * nodes 'agree' that token == 0. This will change when one of them randomly
   * decides to generate a new one
   */
  etimer_set(&et, NEW_TOKEN_INTERVAL);

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    } else if(etimer_expired(&et)) {
      /* Periodically (and randomly) generate a new token. This will trigger
       * a trickle inconsistency */
      if((random_rand() % (NEW_TOKEN_PROB * node_id)) == 0) {
        token++;
        /** MJB04 **/
        //time_stamp = (unsigned long)clock_time();
        time_stamp = clock_seconds();
      	memset(combined, 0, sizeof(combined));
        sprintf(combined, "%lu %d", time_stamp, token);
        PRINTF("At %lu: Generating a new token %s \n", time_stamp, combined);
        /****/    
        trickle_timer_reset_event(&tt);
				powertrace_print("Power Usage:");
      }
      etimer_set(&et, NEW_TOKEN_INTERVAL);
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
