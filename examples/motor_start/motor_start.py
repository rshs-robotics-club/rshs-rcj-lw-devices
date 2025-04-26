from buildhat import Motor

a = Motor('A')
b = Motor('B')
c = Motor('C')
d = Motor('D')

a.start()
a.stop()

b.start()
b.stop()

c.start()
c.stop()

d.start()
d.stop()

print("done running motors")