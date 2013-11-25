from flow_generator import flow

f = open('CoilFlows.txt','w')
flow(f, 30, 20, 70, 80, 0, 5, 100)
flow(f, 30, 80, 70, 20, 0, 5, 100)
flow(f, 15, 10, 94, 96, 100, 5, 100)
f.close()
