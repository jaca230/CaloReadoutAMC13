#!/bin/bash
ifdown eth5
ifup eth5
route add 192.168.1.32 gw 192.168.3.6 dev eth5
