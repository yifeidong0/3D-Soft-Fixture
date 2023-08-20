import tikzplotlib
import matplotlib.pyplot as plt
import numpy as np

gammas = [0.0, 0.0001, 0.001, 0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0]
# mean = [1., 1.00033799, 1.00360022, 1.03352927, 1.21943764, 1.35783091, 1.65152018, 2.34459856, 3.02143819, 3.46040053, 4.28719612]
# std = [0.00000000e+00, 6.75108270e-05, 8.17881685e-04, 4.60239034e-03, 5.12668687e-02, 7.41669606e-02, 1.45081042e-01, 2.88663971e-01, 4.02318067e-01, 5.61519335e-01, 5.29080502e-01]
mean = [1.0209662, 1.01111591, 1.03156818, 1.04106721, 1.18956622, 1.39317529, 1.83914431, 2.1069266, 3.04482549, 3.61389096, 4.57570309] # with reference potential cost
std = [0.0121167, 0.00984046, 0.01569302, 0.00859045, 0.03322219, 0.06515281, 0.23309843, 0.29483142, 0.72894509, 0.36633783, 0.75166235]

gammas = np.asarray(gammas)
mean = np.asarray(mean)
std = np.asarray(std)

plt.plot(gammas, mean, '-*', color='#31a354', linewidth=2)
plt.fill_between(gammas, mean-std, mean+std, alpha=0.4, color='#31a354')
plt.grid(linestyle='dotted', linewidth=0.4)
plt.xlabel('$\\gamma$', fontsize=17)
plt.xlim([1e-4,1])
plt.ylabel('$\\overline{{C}}(\\hat{{\\sigma}}^*)$', fontsize=17)
plt.ylim([0.95,5.00])
plt.yticks([1,2,3,4])
plt.gca().set_xscale('log')
# plt.gca().set_yscale('log')
plt.savefig("gamma-normalized-cost.png")
tikzplotlib.save("gamma-normalized-cost.tex")
# plt.show()