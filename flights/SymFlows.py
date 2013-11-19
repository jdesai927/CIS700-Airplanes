from flow_generator import flow

f = open('SymFlows.txt','w')
flow(f, 20, 45, 80, 45, 0, 5, 100)
flow(f, 80, 55, 20, 55, 0, 5, 100)
flow(f, 45, 15, 45, 85, 0, 5, 100)
flow(f, 55, 85, 55, 15, 0, 5, 100)
f.close()
