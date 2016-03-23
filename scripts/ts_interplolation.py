#!/usr/bin/env python

import numpy as np
import random
import jsonpickle

T =  [30, 60, 120, 180, 240, 300] 
N = [5, 10, 20, 30, 40, 50, 100, 250, 500] #, 1000, 2000] 
M = [5, 10, 15, 20, 30, 50, 100, 125]

INPUT_FILE_G4S =   './Surface-G4SBESTM.json'
INPUT_FILE_IRLAB = './Surface-IRLABBESTM.json'
INPUT_FILE_ALOOF = './Surface-ALOOFBESTM.json'

g4s = dict()
with open(INPUT_FILE_G4S, "r") as input_file:
    json_data = input_file.read()
    g4s = jsonpickle.decode(json_data)

irlab = dict()
with open(INPUT_FILE_IRLAB, "r") as input_file:
    json_data = input_file.read()
    irlab = jsonpickle.decode(json_data)

aloof = dict()
with open(INPUT_FILE_ALOOF, "r") as input_file:
    json_data = input_file.read()
    aloof = jsonpickle.decode(json_data)

surface = dict()
for t in T:
    for n in N:
        for m in M:
            tx_g4s = None
            r_g4s = None
            c_g4s = None
            rc_g4s = None
            r_irlab = None
            c_irlab = None
            rc_irlab = None
            tx_irlab = None
            r_aloof = None
            c_aloof = None
            rc_aloof = None
            tx_aloof = None
            for tx in T:
                tu = (t,n,m,tx,'R')
                if str(tu) in aloof:
                    r_aloof = aloof[str((t,n,m,tx,'R'))]
                    c_aloof = aloof[str((t,n,m,tx,'C'))]
                    rc_aloof = aloof[str((t,n,m,tx,'R/C'))]
                    tx_aloof = tx
                if str(tu) in g4s:
                    r_g4s = g4s[str((t,n,m,tx,'R'))]
                    c_g4s = g4s[str((t,n,m,tx,'C'))]
                    rc_g4s = g4s[str((t,n,m,tx,'R/C'))]
                    tx_g4s = tx
                if str(tu) in irlab:
                    r_irlab = irlab[str((t,n,m,tx,'R'))]
                    c_irlab = irlab[str((t,n,m,tx,'C'))]
                    rc_irlab = irlab[str((t,n,m,tx,'R/C'))]
                    tx_irlab = tx

            tx_min = tx_irlab # np.mean([tx_g4s,tx_irlab,tx_aloof])

            # INTERPOLATION
            # tx_min =  min(tx_g4s,tx_irlab) + float((max(tx_g4s,tx_irlab)-min(tx_g4s,tx_irlab)))/2
            r  = np.mean([r_irlab]) #[r_g4s, r_irlab, r_aloof])
            c  = np.mean([r_irlab]) #[c_g4s, c_irlab, c_aloof])
            rc = np.mean([r_irlab]) #[rc_g4s, rc_irlab, rc_aloof])
            
            surface[(t,n,m,tx_min,'R')] = r
            surface[(t,n,m,tx_min,'C')] = c
            surface[(t,n,m,tx_min,'R/C')] = rc

policy = dict()
maxval = 0
for time in T:
    for (t,n,m,tx,k),val in surface.iteritems():
        if time == t and k == 'R':
            if val >= maxval:
                maxval = val
                policy[time] = (n,m,tx)
                
            
    print time, policy[time]
