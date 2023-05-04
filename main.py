import pandas as pd
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
from sklearn import svm


def train_and_test_model(type_of_model, **kwargs):
    data = pd.read_csv("exported_train_data.csv")
    test_data = pd.read_csv("exported_test_data.csv")
    feature_cols = list(data.columns.values)

    feature_cols.remove('gesture')

    X = data[feature_cols]
    Y = data['gesture']

    X_test = test_data[feature_cols]
    Y_test = test_data['gesture']
    clf = None
    if type_of_model == 'decision_tree':
        clf = DecisionTreeClassifier()
    elif type_of_model == 'random_forest':
        clf = RandomForestClassifier()
    elif type_of_model == 'knn':
        clf = KNeighborsClassifier(n_neighbors=kwargs['n_neighbors'])
    elif type_of_model == 'mlp':
        clf = MLPClassifier(max_iter=data.size, random_state=kwargs['random_state'])
    elif type_of_model == 'svm':
        clf = svm.SVC(kernel="linear")
    clf = clf.fit(X, Y)

    y_pred = clf.predict(X)

    print('Accuracy of {}: {:10.4f}'.format(type_of_model, clf.score(X_test, Y_test)))



def main():
    train_and_test_model(type_of_model="decision_tree")
    train_and_test_model(type_of_model="random_forest")
    train_and_test_model(type_of_model="knn", n_neighbors=3)
    train_and_test_model(type_of_model="mlp", random_state=1)
    train_and_test_model(type_of_model="svm")




main()

