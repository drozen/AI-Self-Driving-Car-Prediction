import math, sys

def dist(point1, point2):
	x1, y1 = point1[0], point1[1]
	x2, y2 = point2[0], point2[1]
	return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


actual = open( sys.argv[1] )
prediction = open( sys.argv[2] )

a = []
p = []

while True:
	l = actual.readline().strip()
	if l:
		x, y = l.split(',')
		a.append( (float(x),float(y)) )
	else:
		break
	l = prediction.readline().strip()
	x, y = l.split(',')
	p.append( (float(x),float(y)) )

d = 0.0
for i in xrange( len(a) ):
	d += (dist( a[i], p[i] ))**2

print math.sqrt( d )




