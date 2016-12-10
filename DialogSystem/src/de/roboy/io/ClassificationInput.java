package de.roboy.io;

import java.util.ArrayList;
import java.util.HashMap;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;

public class ClassificationInput implements Input {
	private HashMap<String, Double> probabilities;

	public ClassificationInput(HashMap probabilities) {
		this.probabilities = probabilities;
	}
}
