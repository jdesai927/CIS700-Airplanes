from flow_generator import flow

f = open('NonSymFlows.txt','w')
flow(f, 20, 20, 65, 30, 0, 5, 100)
flow(f, 35, 15, 80, 15, 0, 5, 100)
flow(f, 20, 40, 65, 50, 0, 5, 100)
flow(f, 35, 35, 80, 35, 0, 5, 100)
flow(f, 40, 5, 40, 60, 100, 20, 12)
flow(f, 40, 60, 40, 5, 100, 20, 12)
f.close()
