#!/usr/bin/env python

import re
import sys

term = sys.argv[1]
inputfile = open("batteryLog.txt", "r")

for line in inputfile:
    if re.match(term, line):
        print line.split(" ")[1],
