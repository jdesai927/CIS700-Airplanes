
def flow(f, x1, y1, x2, y2, t, inc, n):
  for i in range(0,n):
    f.write(str(x1) + ',' + str(y1) + ';' + str(x2) + ',' + str(y2) + ';' + str(t) + '\n')
    t = t + inc

