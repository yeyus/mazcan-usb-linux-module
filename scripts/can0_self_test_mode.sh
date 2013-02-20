#!/bin/sh
sudo ip link set can0 type can tq 100 prop-seg 0 phase-seg1 13 phase-seg2 6 sjw 4
sudo ip link set can0 type can loopback on
sudo ip link set can0 up
