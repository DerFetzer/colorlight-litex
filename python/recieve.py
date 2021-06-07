import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('192.168.1.50', 1234))

msg = s.recv(4)
s.close()
val = int.from_bytes(msg, byteorder='big', signed=False)
print([x for x in msg])
print(val)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []



def animate(i, xs, ys):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('192.168.1.50', 1234))
    msg = s.recv(4)
    s.close()
    val = int.from_bytes(msg, byteorder='big', signed=False)
    print([x for x in msg])
    print(val)
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(val)
     # Limit x and y lists to 100 items
    xs = xs[-200:]
    ys = ys[-200:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('SoCstream')
    plt.ylabel('Temperature')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=500)
plt.show()