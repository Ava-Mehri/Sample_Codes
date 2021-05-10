#Ava Mehri, UIC, amehri2@uic.edu, 2021
#Image Compressor
#Designed and optimized an image compressor from scratch implementing competitive learning algorithm

# -*- coding: utf-8 -*-


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imread
import matplotlib.cm as cm

image = imread('barbara.png')

plt.figure()
plt.imshow(image, cmap='gray')

A = np.matrix(image)
A = A - A.min()
A = (255 / A.max()) * A

plt.figure()
plt.imshow(A, cmap='gray')


def train(epochs, n, K, A, X):
    A_prime = np.zeros((len(A), len(A)))
    RMS_error = []
    winners = []
    losers = []
    for e in range(epochs):
        # Analaysing the RMS error array, I observed that after 2 epochs it remains constant, meaning that some neurons never win
        # To solve it, after epoch = 2, I reinitialized the weights of those neurons.
        if e == 3 or e == 5:
            for p in range(K):
                if not p in winners:
                    X[p] = A_junior
        for i in range(n):
            for j in range(n):
                ED = []
                A_junior = A[4 * i: 4 * i + 4, 4 * j: 4 * j + 4]
                for k in range(K):
                    X[k] = np.matrix(X[k])
                    ed = np.sum(np.square(A_junior - X[k]))
                    ED.append(ed)
                k_star = np.argmin(ED)
                winners.append(k_star)
                X[k_star] = (1 - eta) * X[k_star] + eta * A_junior
                A_prime[4 * i: 4 * i + 4, 4 * j: 4 * j + 4] = X[k_star]
        RMS_error.append((1 / len(A)) * np.linalg.norm(A_prime - A))
        # RMS_error.append((1/len(A))*np.sqrt(np.sum(np.square(A_prime - A))))

    # I used this for loop to verify that losers array would be empty, meaning that all neurons have won at least once.
    # Showing that losers array is empty verifies that my code would defiently return the least RMS.
    for q in range(K):
        if not q in winners:
            losers.append(q)
    return X, RMS_error, losers


def test(n, K, A, X):
    A_prime = np.zeros((len(A), len(A)))
    for i in range(n):
        for j in range(n):
            ED = []
            A_junior = A[4 * i: 4 * i + 4, 4 * j: 4 * j + 4]
            for k in range(K):
                X[k] = np.matrix(X[k])
                ed = np.sum(np.square(A_junior - X[k]))
                ED.append(ed)
            k_star = np.argmin(ED)
            A_prime[4 * i: 4 * i + 4, 4 * j: 4 * j + 4] = X[k_star]

    RMS_error = (1 / len(A)) * np.linalg.norm(A_prime - A)
    # RMS_error = (1/len(A))*np.sqrt(np.sum(np.square(A_prime - A)))
    return A_prime, RMS_error


d = 4
K = 8
n = int(len(A) / d)
eta = 0.5
epochs = 10

X = np.random.uniform(0, 255, size=(1, K * d * d))
X = X.reshape(K, d, d)

X_star, RMS_error_train, losers = train(epochs, n, K, A, X)
A_prime, RMS_error_test = test(n, K, A, X_star)

print(f'Training RMS error after each epoch = {RMS_error_train}\nLoser neurons = {losers}')
plt.figure()
plt.plot(RMS_error_train)
plt.xlabel('number of epochs')
plt.ylabel('RMS error')
plt.title(f'Training RMS error vs epochs for K = {K}')

print(f'Test RMS error = {RMS_error_test}')
plt.figure()
plt.imshow(A_prime, cmap='gray')
plt.title(f'Compressed image for K = {K}')

## This part is the exact same procedure but for different Ks, uncomment it if you want to run it and verify the results in my report.

# d = 4
# K = [2, 8, 32]
# n = int(len(A) / d)
# eta = 0.5
# epochs = 10

# for k in K:
#  X = np.random.uniform(0, 255, size=(1, k * d * d))
#  X = X.reshape(k, d, d)

#  X_star, RMS_error_train, losers = train(epochs, n, k, A, X)
#  A_prime, RMS_error_test = test(n, k, A, X_star)

#  print(f'Training RMS error for K = {k} after each epoch = {RMS_error_train}\nLoser neurons = {losers}')
#  plt.figure()
#  plt.plot(RMS_error_train)
#  plt.xlabel('number of epochs')
#  plt.ylabel('RMS error')
#  plt.title(f'Training RMS error vs epochs for K = {k}')

#  print(f'Test RMS error (for K = {k}) = {RMS_error_test}')
#  plt.figure()
#  plt.imshow(A_prime, cmap='gray')
#  plt.title(f'Compressed image for K = {k}')


plt.show()
