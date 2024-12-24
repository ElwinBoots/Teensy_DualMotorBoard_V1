m.setpar('motor.state1.Valpha_offset', 0.1)
time.sleep(0.5)
Valpha1 = m.getsig('motor.state1.Valpha_offset')
Ialpha1 = m.getsig('motor.state1.Ialpha')
Ibeta1 = m.getsig('motor.state1.Ibeta')
Ia1 = m.getsig('motor.state1.ia')
bus1 = m.getsig('motor.state.sensBus_lp')

m.setpar('motor.state1.Valpha_offset', 0.2)

time.sleep(1)
Valpha2 = m.getsig('motor.state1.Valpha_offset')
Va = m.getsig('motor.state1.Va')
Vb = m.getsig('motor.state1.Vb')
Ialpha2 = m.getsig('motor.state1.Ialpha')
Ibeta2 = m.getsig('motor.state1.Ibeta')
bus2 = m.getsig('motor.state.sensBus')
Ia2 = m.getsig('motor.state1.ia')
m.setpar('motor.state1.Valpha_offset', 0)

R = (Valpha2 - Valpha1) / (Ialpha2 - Ialpha1)


#%%

m.setpar('motor.state2.Valpha_offset', 0.5)
time.sleep(0.5)
Valpha1 = m.getsig('motor.state2.Valpha_offset')
Ialpha1 = m.getsig('motor.state2.Ialpha')
Ibeta1 = m.getsig('motor.state2.Ibeta')
Ia1 = m.getsig('motor.state2.ia')
bus1 = m.getsig('motor.state.sensBus_lp')

m.setpar('motor.state2.Valpha_offset', 1)

time.sleep(1)
Valpha2 = m.getsig('motor.state2.Valpha_offset')
Va = m.getsig('motor.state2.Va')
Vb = m.getsig('motor.state2.Vb')
Ialpha2 = m.getsig('motor.state2.Ialpha')
Ibeta2 = m.getsig('motor.state2.Ibeta')
bus2 = m.getsig('motor.state.sensBus')
Ia2 = m.getsig('motor.state2.ia')
m.setpar('motor.state2.Valpha_offset', 0)

R = (Valpha2 - Valpha1) / (Ialpha2 - Ialpha1)