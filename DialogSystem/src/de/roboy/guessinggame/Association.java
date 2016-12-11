package de.roboy.guessinggame;

import java.util.List;

public class Association {
	private List<String> nouns;
	private List<String> verbs;
	private List<String> adjectives;

	public Association(List<String> nouns, List<String> verbs, List<String> adjectives) {
		this.nouns = nouns;
		this.verbs = verbs;
		this.adjectives = adjectives;
	}

	public double getProbability(List<String> hints) { // TODO which parameters?
		double result, position, length;
		result = 0;
		for (int i = 0; i < hints.size(); ++i) {
			if (nouns.contains((hints.get(i)))) {
				position = nouns.indexOf(hints.get(i));
				length = nouns.size();
			} else if (verbs.contains((hints.get(i)))) {
				position = verbs.indexOf(hints.get(i));
				length = verbs.size();
			} else if (adjectives.contains((hints.get(i)))) {
				position = adjectives.indexOf(hints.get(i));
				length = adjectives.size();
			} else {
				continue;
			}
			result += 1;
		}
		//result = result / hints.size();
		return result;
	}
}
