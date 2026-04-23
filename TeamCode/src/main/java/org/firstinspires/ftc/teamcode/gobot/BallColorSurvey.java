package org.firstinspires.ftc.teamcode.gobot;

import java.util.List;

public class BallColorSurvey {
    private final int[][] colorVotes = new int[3][3];
    private int sampleCount = 0;
    private int targetSamples; // This is what the '5' or '3' sets

    // ADD THIS CONSTRUCTOR
    public BallColorSurvey(int initialTarget) {
        this.sampleCount = 0;
        this.targetSamples = initialTarget;
    }

    // ADD THIS RESET METHOD
    public void reset(int newTarget) {
        for (int i = 0; i < 3; i++) {
            colorVotes[i][0] = 0; // None
            colorVotes[i][1] = 0; // Red
            colorVotes[i][2] = 0; // Blue
        }
        this.sampleCount = 0;
        this.targetSamples = newTarget;
    }

    public void addSample(List<Character> currentRead) {
        for (int i = 0; i < 3; i++) {
            char c = currentRead.get(i);
            int colIdx = (c == 'R') ? 1 : (c == 'B' ? 2 : 0);
            colorVotes[i][colIdx]++;
        }
        sampleCount++;
    }

    public boolean isComplete() {
        return sampleCount >= targetSamples;
    }

    public Character[] getResult() {
        Character[] finalColors = new Character[3];
        for (int i = 0; i < 3; i++) {
            int maxIdx = 0;
            if (colorVotes[i][1] > colorVotes[i][0]) maxIdx = 1;
            if (colorVotes[i][2] > colorVotes[i][maxIdx]) maxIdx = 2;

            finalColors[i] = (maxIdx == 1) ? 'R' : (maxIdx == 2 ? 'B' : 'N');
        }
        return finalColors;
    }

    public int getSampleCount() { return sampleCount; }
}
