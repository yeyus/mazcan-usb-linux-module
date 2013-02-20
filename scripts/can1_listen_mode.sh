#!/bin/sh
sudo ip link set can1 type can tq 400 prop-seg 0 phase-seg1 13 phase-seg2 6 sjw 4
sudo ip link set can1 type can listen-only on
sudo ip link set can1 up
