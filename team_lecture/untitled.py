while True:
	prompt = "%s words: " % 3
	sentence = input(prompt)
	sentence = sentence.strip()
	words = sentence.split(' ')

	print(sentence)