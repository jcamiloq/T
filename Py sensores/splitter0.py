import matplotlib.pyplot as plt
# global lines
array0 = []
both0 = []
# parte1 = []
# parte2 = []
# fileid = ''
array = []
wave = []
inten = []
# resta = []
captcounter = 0
index = 0
wavelenghts = []
blanco = []
negro = []
medida = []
espectro = []
resta = []
suma = []
# saveFile = open("capturazo.txt", 'w')
# saveFile.close()
# for j in range(0, 100):
#     appendFile = open("capturazo.txt", 'a')
#     appendFile.write(str(j) + '\n')
#     appendFile.close()
x = "5vis.txt"


with open(x, "r") as in_file:
    capturas = round(int(len(in_file.readlines())) / 1024)
    in_file.seek(0)
    # print(str(capturas))
    for line in in_file:
        array0.append(line)

    # print(array0)
    for i in range(0, capturas):
        saveFile = open("captura%s.txt" % (i), 'w')
        saveFile.close()
        for j in range(0, 1024):
            index = j + (captcounter)
            appendFile = open("captura%s.txt" % (i), 'a')
            appendFile.write(str(array0[index]))
            appendFile.close()

        # with open("captura%s.txt" % (i), "r") as ins:

        #     for line in ins:
        #         array.append(line)
        #         both = line.split(" ")
        #         wave.append(both[0])
        #         inten.append(both[1])

#         # print(wave)
#         print(inten)
#         # print(array)
#         appendFile = open('inten.txt', 'a')
#         for i in range(0, 1024):
#             appendFile.write(wave[i] + ' ' + inten[i])
#         saveFile.close()

#         for i in range(0, len(wave)):
#             resta.append(float(wave[i]) - float(inten[i]))
#         # print(resta)
#         captcounter = captcounter + 1024
with open("wavevis.txt", "r") as wavevis:
    for line in wavevis:
        # line = line[:-4]
        wavelenghts.append(line)
# blanco-negro/medida-negro
with open("DataVISb'w'2019-02-14 16-46-04.80598220 5.txt", "r") as blancovis:
    for line in blancovis:
        line = line[0:4]
        blanco.append(line)
with open("DataVISb'n'2019-02-14 16-43-47.76328120 5.txt", "r") as negrovis:
    for line in negrovis:
        line = line[0:4]
        negro.append(line)
with open(x, "r") as grass:
    for line in grass:
        line = line[0:4]
        medida.append(line)

for i in range(0, len(wavelenghts)):
    suma.append(float(blanco[i]) - float(negro[i]))
# print(str(resta))
for i in range(0, len(wavelenghts)):
    resta.append(float(medida[i]) - float(negro[i]))
# print(str(suma))
for i in range(0, len(wavelenghts)):
    if suma[i] == 0:
        suma[i] = 1
    espectro.append(float(resta[i]) / float(suma[i]))

ax1 = plt.subplot(231)
plt.plot(wavelenghts, medida, label='Datos Brutos')
plt.legend()
# plt.axis([0, 82, 0, 50])
ax2 = plt.subplot(232, sharey=ax1)
plt.plot(wavelenghts, negro, label='Negro')
plt.legend()
ax3 = plt.subplot(233, sharey=ax1)
plt.plot(wavelenghts, blanco, label='Blanco')
# Add a legend
plt.legend()
ax4 = plt.subplot(234, sharey=ax1)
plt.plot(wavelenghts, espectro, label='Espectro')
# Add a legend
plt.legend()
wm = plt.get_current_fig_manager()
wm.window.state('zoomed')
# Show the plot
plt.show()
