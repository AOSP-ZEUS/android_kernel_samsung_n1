/*
 * Copyright (C) 2007-2011 B.A.T.M.A.N. contributors:
 *
 * Marek Lindner, Simon Wunderlich
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of version 2 of the GNU General Public
 * License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 *
 */

#ifndef _NET_BATMAN_ADV_AGGREGATION_H_
#define _NET_BATMAN_ADV_AGGREGATION_H_

#include "main.h"

/* is there another aggregated packet here? */
<<<<<<< HEAD
static inline int aggregated_packet(int buff_pos, int packet_len, int num_tt)
{
	int next_buff_pos = buff_pos + BAT_PACKET_LEN + (num_tt * ETH_ALEN);
=======
static inline int aggregated_packet(int buff_pos, int packet_len,
				    int tt_num_changes)
{
	int next_buff_pos = buff_pos + BAT_PACKET_LEN + (tt_num_changes *
						sizeof(struct tt_change));
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

	return (next_buff_pos <= packet_len) &&
		(next_buff_pos <= MAX_AGGREGATION_BYTES);
}

void add_bat_packet_to_list(struct bat_priv *bat_priv,
			    unsigned char *packet_buff, int packet_len,
<<<<<<< HEAD
			    struct hard_iface *if_incoming, char own_packet,
			    unsigned long send_time);
void receive_aggr_bat_packet(struct ethhdr *ethhdr, unsigned char *packet_buff,
			     int packet_len, struct hard_iface *if_incoming);
=======
			    struct hard_iface *if_incoming, int own_packet,
			    unsigned long send_time);
void receive_aggr_bat_packet(const struct ethhdr *ethhdr,
			     unsigned char *packet_buff, int packet_len,
			     struct hard_iface *if_incoming);
>>>>>>> 0c0a7df444663b2da5ce70e9b9129a9cfe1b07c7

#endif /* _NET_BATMAN_ADV_AGGREGATION_H_ */
