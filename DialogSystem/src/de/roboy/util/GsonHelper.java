package de.roboy.util;

import java.awt.List;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import de.roboy.dialog.action.Action;
import de.roboy.dialog.action.EmotionAction;
import de.roboy.dialog.action.MovementAction;
import de.roboy.dialog.action.SpeechAction;
import de.roboy.io.ClassificationInput;
import de.roboy.io.Input;
import de.roboy.io.SentenceInput;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

public class GsonHelper {
	public static ArrayList<Input> decode(String input) {
		// TODO
		if(input.equals("")){
			return null;
		}
		Gson gson = new GsonBuilder().create();
		JsonElement jelem = gson.fromJson(input, JsonElement.class);
		JsonArray jarr = jelem.getAsJsonArray();

		ArrayList<Input> inputList = new ArrayList<Input>();
		for (JsonElement jel : jarr) {
			if (jel.getAsJsonObject().has("text")) {
				inputList.add(new SentenceInput(jel.getAsJsonObject().get("text").getAsString()));
			} else {
				HashMap<String, Double> foo = new HashMap<String, Double>();
				for (Entry<String, JsonElement> jelCla : jel.getAsJsonObject().get("imagenet").getAsJsonObject()
						.entrySet()) {
					foo.put(jelCla.getKey(), jelCla.getValue().getAsDouble());
				}
				inputList.add(new ClassificationInput(foo));
			}

		}
		return inputList;
	}

	public static String encode(ArrayList<Action> actionList) {
		// TODO
		String jsonString;
		Gson gson = new GsonBuilder().create();
		JsonArray jarr = new JsonArray();
		for(Action action : actionList){
			if(action.getClass() == SpeechAction.class){
				JsonObject obj = new JsonObject();
				obj.addProperty("speak", ((SpeechAction)action).getText());
				jarr.add(obj);
			}
			if(action.getClass() == EmotionAction.class){
				JsonObject obj = new JsonObject();
				obj.addProperty("emotion", ((EmotionAction)action).getEmotion());
				jarr.add(obj);
			}
			if(action.getClass() == MovementAction.class){
				JsonObject obj = new JsonObject();
				obj.addProperty("movement", ((MovementAction)action).getMovement());
				jarr.add(obj);
			}
		}
		jsonString = jarr.getAsString();
		return jsonString;
	}
}
