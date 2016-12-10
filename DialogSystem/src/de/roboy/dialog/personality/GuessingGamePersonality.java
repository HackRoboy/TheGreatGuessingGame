package de.roboy.dialog.personality;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import de.roboy.dialog.action.Action;
import de.roboy.dialog.action.EmotionAction;
import de.roboy.dialog.action.ShutDownAction;
import de.roboy.dialog.action.SpeechAction;
import de.roboy.io.ClassificationInput;
import de.roboy.io.Input;
import de.roboy.io.SentenceInput;
import de.roboy.linguistics.sentenceanalysis.Interpretation;
import de.roboy.util.GsonHelper;

public class GuessingGamePersonality implements Personality {

	private enum GuessingGameState {
		WELCOME, REGISTER_GROUP_A, REGISTER_GROUP_B, REGISTER_TERM, REGISTER_STOP_WORDS, FINALIZE_SETUP, GROUP_A_SENTENCE_INPUT, GROUP_B_SENTENCE_INPUT, TELL_RESULTS
	}

	private GuessingGameState state = GuessingGameState.WELCOME;
	private String groupA = "";
	private String groupB = "";
	private String term = "";
	private HashSet<String> stopWords = new HashSet<String>();

	@Override
	public List<Action> answer(Interpretation input) {
		List<Action> result = new ArrayList<Action>();
		List<Input> listInput = GsonHelper.decode(input.sentence);
		SentenceInput sentence = null;
		ClassificationInput classification = null;
		if (listInput != null) {
			for (Input i : listInput) {
				if (i.getClass() == SentenceInput.class) {
					// TODO multiple Inputs?
					sentence = (SentenceInput) i;
				} else {
					classification = (ClassificationInput) i;
				}
			}
		}
		/*
		 * At the beginning of a state the input for the previous state gets
		 * processed and if the input is not valid, it repeats this state
		 */
		switch (state) {
		case WELCOME:
			// TODO more introduction to the game
			result.add(new SpeechAction("Welcome to the Guessing Game with Roboy"));
			state = GuessingGameState.REGISTER_GROUP_A;
			// return result;
		case REGISTER_GROUP_A:
			// Ask groupA for groupname
			result.add(new SpeechAction("Group 1, please tell me the name of your group."));
			state = GuessingGameState.REGISTER_GROUP_B;
			return result;
		case REGISTER_GROUP_B:
			// save and greet groupA
			if (sentence != null) {
				if (groupA.equals("")) {
					if (sentence.getInput().length() <= 30) {
						groupA = sentence.getInput();
						result.add(new SpeechAction("Group 1 will now be called " + groupA));
					} else {
						result.add(new SpeechAction("Group 1, the name is too long. Please choose a shorter one."));
						state = GuessingGameState.REGISTER_GROUP_B;
						return result;
					}
				}
			} else {
				result.add(new SpeechAction("Invalid Input.\nGroup 1, please choose a groupname."));
				state = GuessingGameState.REGISTER_GROUP_B;
				return result;
			}

			// ask groupB for groupname
			result.add(new SpeechAction("Group 2, please tell me the name of your group."));
			state = GuessingGameState.REGISTER_TERM;
			return result;

		case REGISTER_TERM:
			// save and greet groupB
			if (sentence != null) {
				if (groupB.equals("")) {
					if (sentence.getInput().length() <= 30) {
						groupB = sentence.getInput();
						result.add(new SpeechAction("Group 2 will now be called " + groupB));
					} else {
						result.add(new SpeechAction("Group 2, the name is too long. Please choose a shorter one."));
						state = GuessingGameState.REGISTER_TERM;
						return result;
					}
				}
			} else {
				result.add(new SpeechAction("Invalid Input.\nGroup 2, please choose a groupname."));
				state = GuessingGameState.REGISTER_TERM;
				return result;
			}

			// Ask for Term
			result.add(new SpeechAction("Please tell me the object yout want me to guess."));
			state = GuessingGameState.REGISTER_STOP_WORDS;
			return result;

		case REGISTER_STOP_WORDS:
			// check input for term
			if (sentence == null) {
				result.add(new SpeechAction("You did not say an object. You don't want to play with me."));
				result.add(new EmotionAction("sad"));
				result.add(new SpeechAction("Try again. Please tell me the object yout want me to guess."));
				state = GuessingGameState.REGISTER_STOP_WORDS;
				return result;
			}
			if (classification != null) {
				if (term.equals("")) {
					int iSeeIt;
					if (classification.getProbabilities().containsKey(sentence.getInput())) {
						term = sentence.getInput();
						result.add(
								new SpeechAction("I see " + term + ". But I will forget that you told me what it is."));
						result.add(new EmotionAction("happy"));// TODO make him
																// twinkle
					} else {
						result.add(new SpeechAction("I can't see " + sentence.getInput()));
						result.add(new EmotionAction("sad"));
						result.add(
								new SpeechAction("Let me see again. Please tell me the object yout want me to guess."));
						state = GuessingGameState.REGISTER_STOP_WORDS;
						return result;
					}
				}
			} else {
				if (term.equals("")) {
					result.add(new SpeechAction("I can't see anything"));
					result.add(new EmotionAction("sad"));
					result.add(new SpeechAction("Let me see again. Please tell me the object yout want me to guess."));
					state = GuessingGameState.REGISTER_STOP_WORDS;
					return result;
				}
			}

			// Ask for stopwords
			result.add(new SpeechAction("Please tell me words that should not be used to describe the object."));
			state = GuessingGameState.FINALIZE_SETUP;
			return result;

		case FINALIZE_SETUP:
			// save the stopwords
			if (sentence != null) {
				String[] arr = sentence.getInput().split("\\s+|,"); // TODO
																	// regex
				// the term is also not allowed to say
				stopWords.add(term);
				String allStopWords = term;
				for (int i = 0; i < arr.length; ++i) {
					arr[i].replaceAll("\\s+|,", "");
					stopWords.add(arr[i]);
					allStopWords = allStopWords + ", " + arr[i];
				}
				result.add(new SpeechAction("The words you are not allowed to use are: " + allStopWords + "."));
			}
			result.add(new SpeechAction(groupA + ", please describe the object."));
			state = GuessingGameState.GROUP_A_SENTENCE_INPUT;
			return result;
		case GROUP_A_SENTENCE_INPUT:
		case GROUP_B_SENTENCE_INPUT:
		case TELL_RESULTS:
		default:
			List<Action> lastwords = new ArrayList<Action>();
			lastwords.add(new SpeechAction("I am lost, bye."));
			result.add(new ShutDownAction(lastwords));
			return result;
		}
	}

	private void filterSpamWords() {
		// TODO take out words like 'and', 'or', .. of the sentence and return
		// the filtered sentence
		// filter with POS: UH, ..
	}

}
