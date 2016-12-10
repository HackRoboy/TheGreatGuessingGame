package de.roboy.dialog;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;

import de.roboy.dialog.action.Action;
import de.roboy.dialog.action.ShutDownAction;
import de.roboy.dialog.personality.CuriousPersonality;
import de.roboy.dialog.personality.DefaultPersonality;
import de.roboy.dialog.personality.GuessingGamePersonality;
import de.roboy.dialog.personality.KnockKnochPersonality;
import de.roboy.dialog.personality.Personality;
import de.roboy.dialog.personality.SmallTalkPersonality;
import de.roboy.io.ClassificationInput;
import de.roboy.io.CommandLineCommunication;
import de.roboy.io.CommandLineInput;
import de.roboy.io.CommandLineOutput;
import de.roboy.io.Communication;
import de.roboy.io.Input;
import de.roboy.io.InputDevice;
import de.roboy.io.OutputDevice;
import de.roboy.io.SentenceInput;
import de.roboy.linguistics.sentenceanalysis.Analyzer;
import de.roboy.linguistics.sentenceanalysis.Interpretation;
import de.roboy.linguistics.sentenceanalysis.SentenceAnalyzer;
import de.roboy.util.GsonHelper;

public class DialogSystem {
	
	public static void main(String[] args) throws JsonSyntaxException, JsonIOException, IOException {
//		Personality p = new DefaultPersonality();
//		Personality p = new CuriousPersonality();
//		Personality p = new KnockKnochPersonality();
//		Personality p = new SmallTalkPersonality();
		Personality p = new GuessingGamePersonality();
		
		InputDevice input = new CommandLineInput();
		OutputDevice output = new CommandLineOutput();
		Analyzer analyzer = new SentenceAnalyzer();
		
		String raw; //  = input.listen();
		Interpretation interpretation; // = analyzer.analyze(raw);
		List<Action> actions =  p.answer(new Interpretation(""));
		while(actions.size()>=1 && !(actions.get(0) instanceof ShutDownAction)){
			output.act(GsonHelper.encode(actions));
			raw = input.listen();
			//TODO raw is in json -> only interpret "text:"
			List<Input> listInput = GsonHelper.decode(raw);
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
			if(sentence == null){
				continue;
			}
			interpretation = analyzer.analyze(sentence.getInput());
			if (p.getClass() == GuessingGamePersonality.class) {
				((GuessingGamePersonality) p).setClassification(classification);
				((GuessingGamePersonality) p).setSentence(sentence);
				
			}
			actions = p.answer(interpretation);
		}
		List<Action> lastwords = ((ShutDownAction)actions.get(0)).getLastWords();
		output.act(lastwords);
	}

}
