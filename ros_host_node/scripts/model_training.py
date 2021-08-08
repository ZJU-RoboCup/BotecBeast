from keras.datasets import mnist
from keras import layers
from keras import models
import numpy as np
import rospy
import rospkg

def get_data():
    (train_images, train_labels), (test_images, test_labels) = mnist.load_data()

    train_images = train_images.reshape((train_images.shape[0], 28, 28, 1)).astype('float32') / 255
    test_images = test_images.reshape((test_images.shape[0], 28, 28, 1)).astype('float32') / 255
    
    def to_label(labels):
        zero_labels = np.zeros((labels.shape[0], 10), dtype=int)
        for index, label in enumerate(labels):
            zero_labels[index][label] = 1
        return zero_labels

    train_labels = to_label(train_labels)    
    test_labels = to_label(test_labels)    

    return (train_images, train_labels), (test_images, test_labels)


if __name__ == "__main__":
    rospy.init_node("model_training")
    pkg_path = rospkg.RosPack().get_path("ros_host_node")

    (train_images, train_labels), (test_images, test_labels) = get_data()

    model = models.Sequential()
    model.add(layers.Conv2D(32, (3,3), activation='relu', input_shape=(28,28,1)))
    model.add(layers.MaxPooling2D(2,2))
    model.add(layers.Conv2D(64, (3,3), activation='relu'))
    model.add(layers.MaxPooling2D((2,2)))
    model.add(layers.Conv2D(64, (3,3), activation='relu'))
    model.add(layers.Flatten())
    model.add(layers.Dense(64, activation='relu'))
    model.add(layers.Dense(10, activation='softmax'))
    model.summary()
    json_str = model.to_json()
    with open(pkg_path + "/scripts/model.json", "w") as f:
        f.write(json_str)
    

    model.compile(optimizer='rmsprop', loss='categorical_crossentropy', metrics=['accuracy'])
    model.fit(train_images, train_labels, epochs = 5, batch_size=64)

    test_loss, test_acc = model.evaluate(test_images, test_labels)  
    print("test_loss: {}, test_acc: {}".format(test_loss, test_acc))
    model.save(pkg_path + "/scripts/model.h5")
