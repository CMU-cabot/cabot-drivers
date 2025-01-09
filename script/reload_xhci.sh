#!/bin/bash

sudo modprobe -r xhci_pci

sleep 8

sudo modprobe xhci_pci

sleep 2