
def flow(x1, y1, x2, y2, t, inc, n):
  for i in range(0,n):
    f.write(str(x1) + ',' + str(y1) + ';' + str(x2) + ',' + str(y2) + ';' + str(t) + '\n')
    t = t + inc

f = open('test-flows.txt','w')
flow(20, 20, 80, 50, 0, 5, 100)
flow(80, 50, 20, 20, 0, 5, 100)
flow(20, 50, 80, 20, 0, 5, 100)
flow(80, 20, 20, 50, 0, 5, 100)
flow(50, 10, 50, 80, 0, 5, 100)
flow(50, 80, 50, 10, 0, 5, 100)
f.close()

