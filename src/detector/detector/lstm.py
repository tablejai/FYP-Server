import numpy as np
import pandas as pd
import tensorflow as tf
import os

from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.preprocessing import sequence
from keras.models import Sequential
from keras.layers import Activation, Dense, Dropout, LSTM, Softmax, Bidirectional
from sklearn.model_selection import train_test_split

"""#Load & Prepare Data"""

# directory = "/content/drive/MyDrive/Colab Notebooks/rosbag/labeled_data/labeled_data_30hz"
directory = "/home/ubuntu/FYP-ROS/rosbag/data"

# get x
X = []
for filename in os.listdir(f"{directory}/data"):
    f = os.path.join(f"{directory}/data", filename)
    if os.path.isfile(f):
        x_raw = pd.read_csv(f)
        x_raw["timestamp"] -= x_raw["timestamp"][0]
        x_raw = x_raw.to_numpy()
        x_raw = np.pad(x_raw, ((0, 500 - x_raw.shape[0]), (0, 0)), 'constant')
        X.append(x_raw)

X = np.array(X)

# get y
y = []
for filename in os.listdir(f"{directory}/label"):
    f = os.path.join(f"{directory}/label", filename)
    if os.path.isfile(f):
        y.append(pd.read_csv(f)['label'])

y = tf.keras.utils.to_categorical(y, num_classes=7)

# print dimension
print(f"{X.shape=}")
print(f"{y.shape=}")

print(f"{X[0].shape=}")
print(f"{X[1].shape=}")

print(f"{X[1]=}")
print(f"{y[100]=}")

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.1, random_state=42)

"""# Build Model"""

model=Sequential()

model.add(Bidirectional(LSTM(units=64, return_sequences=True), input_shape=(500, 45)))
model.add(Bidirectional(LSTM(units=64, return_sequences=True)))
model.add(Bidirectional(LSTM(units=64)))
model.add(Dense(7, activation='softmax'))

model.compile(loss='categorical_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])
print(model.summary())

"""# Training"""

model.fit(X_train, y_train, validation_data=(X_test, y_test),epochs=20, batch_size=10)

"""#Evaluate"""

scores = model.evaluate(X_test, y_test, verbose=0)
print(f"{scores=}")

y_pred = model.predict(X_test[11:12])

print(f"{np.argmax(y_pred)=}")
print(f"{np.argmax(y_test[11:12])=}")