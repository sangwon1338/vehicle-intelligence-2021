  #!/usr/bin/env python3

from classifier import GNB
import json

def main():
	gnb = GNB()
	with open('train.json', 'r') as f:
		j = json.load(f)
	X = j['states']
	Y = j['labels']
	gnb.train(X, Y)

	with open('test.json', 'r') as f:
		j = json.load(f)

	X = j['states']
	Y = j['labels']
	score = 0
	for coords, label in zip(X, Y):
		predicted = gnb.predict(coords)
		if predicted == label:
			score += 1
	fraction_correct = float(score) / len(X)
	print("You got %.2f percent correct" % (100 * fraction_correct))

if __name__ == "__main__":
	main()
