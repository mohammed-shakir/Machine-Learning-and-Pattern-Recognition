import pandas as pd
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.ensemble import BaggingClassifier
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.ensemble import VotingClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.naive_bayes import GaussianNB
from sklearn.decomposition import PCA
from sklearn import svm
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import GridSearchCV
from sklearn.model_selection import cross_val_score
import math

def train_and_test_model(type_of_model, use_pca=False, **kwargs):
    data = pd.read_csv("exported_train_data.csv")
    test_data = pd.read_csv("exported_test_data.csv")
    feature_cols = list(data.columns.values)

    feature_cols.remove('gesture')

    X = data[feature_cols]
    Y = data['gesture']

    X_test = test_data[feature_cols]
    Y_test = test_data['gesture']

    # Normalize data
    scaler = StandardScaler()
    X = scaler.fit_transform(X)
    X_test = scaler.transform(X_test)

    if use_pca:
        pca = PCA(n_components=0.95)
        X = pca.fit_transform(X)
        X_test = pca.transform(X_test)

    clf = None

    if type_of_model == 'decision_tree':
        clf = DecisionTreeClassifier()
    elif type_of_model == 'random_forest':
        clf = RandomForestClassifier()
    elif type_of_model == 'knn':
        length = len(data)
        best_k_value = 0
        best_accuracy = 0
        for i in range(1, int(math.sqrt(length))):
            clf = KNeighborsClassifier(n_neighbors=i)
            clf = clf.fit(X, Y)
            accuracy = clf.score(X_test, Y_test)
            if accuracy > best_accuracy:
                best_accuracy = accuracy
                best_k_value = i
        print('Best k value for knn: {}'.format(best_k_value))
        clf = KNeighborsClassifier(n_neighbors=best_k_value)
    elif type_of_model == 'mlp':
        clf = MLPClassifier(max_iter=data.size, random_state=None)
    elif type_of_model == 'svm':
        clf = svm.SVC(kernel="linear")
    elif type_of_model == 'bagging':
        clf = BaggingClassifier(DecisionTreeClassifier(), n_estimators=100, random_state=0)
    elif type_of_model == 'extra_trees':
        clf = ExtraTreesClassifier(n_estimators=100, random_state=0)
    elif type_of_model == 'voting':
        clf1 = LogisticRegression(multi_class='multinomial', random_state=0, max_iter=data.size)
        clf2 = RandomForestClassifier(n_estimators=100, random_state=0)
        clf3 = GaussianNB()
        clf = VotingClassifier(estimators=[('lr', clf1), ('rf', clf2), ('gnb', clf3)], voting=kwargs["type"])
    
    clf = clf.fit(X, Y)
    y_pred = clf.predict(X)
    print('Accuracy of {}: {:10.4f}'.format(type_of_model, clf.score(X_test, Y_test)))

def main():
    print("--------------------- Task 2 ---------------------")
    train_and_test_model(type_of_model="decision_tree")
    train_and_test_model(type_of_model="random_forest")
    train_and_test_model(type_of_model="knn")
    train_and_test_model(type_of_model="mlp")
    train_and_test_model(type_of_model="svm")
    print("--------------------- Task 3 ---------------------")
    train_and_test_model(type_of_model="bagging")
    train_and_test_model(type_of_model="extra_trees")
    train_and_test_model(type_of_model="voting", type="hard")
    print("--------------------- Task 3 with PCA ---------------------")
    train_and_test_model(type_of_model="bagging", use_pca=True)
    train_and_test_model(type_of_model="extra_trees", use_pca=True)
    train_and_test_model(type_of_model="voting", type="hard", use_pca=True)

main()
