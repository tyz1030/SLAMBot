import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser(description='Plot the sensor model pdf')
parser.add_argument('file', help='file with sensor pdf as a csv')

args = parser.parse_args()

d = []
pdf = []
with open(args.file, 'r') as f:
    for l in f:
        dd, p = (float(val) for val in l.split(','))
        d.append(dd)
        pdf.append(p)

print(len(d))
plt.plot(d, pdf)
plt.xlabel('distance (m)')
plt.ylabel('unnormalized pdf')
plt.show()
