import tikzplotlib
import matplotlib.pyplot as plt
import numpy as np

alphas = [0.0, 0.0001, 0.001, 0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]
mean = [1., 1.00033799, 1.00360022, 1.03352927, 1.21943764, 1.35783091, 1.65152018, 2.34459856, 3.02143819, 3.46040053, 4.28719612]
std = [0.00000000e+00, 6.75108270e-05, 8.17881685e-04, 4.60239034e-03, 5.12668687e-02, 7.41669606e-02, 1.45081042e-01, 2.88663971e-01, 4.02318067e-01, 5.61519335e-01, 5.29080502e-01]

alphas = np.asarray(alphas)
mean = np.asarray(mean)
std = np.asarray(std)

plt.plot(alphas, mean, '-', color='#31a354', linewidth=2)
plt.fill_between(alphas, mean-std, mean+std, alpha=0.4, color='#31a354')
plt.grid(linestyle='dotted', linewidth=0.4)
plt.xlabel("alpha")
plt.xlim([1e-4,1])
plt.ylabel("normalized estimated escape energy")
plt.ylim([0.95,5.00])
plt.yticks([1,2,3,4,5])
plt.gca().set_xscale('log')
plt.savefig("alpha-normalized-cost.png")
tikzplotlib.save("alpha-normalized-cost.tex")
# plt.show()