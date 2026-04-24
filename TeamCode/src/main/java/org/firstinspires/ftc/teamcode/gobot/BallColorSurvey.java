package org.firstinspires.ftc.teamcode.gobot;

import java.util.List;

public class BallColorSurvey {
    private final int[][] colorVotes = new int[3][3];
    private int sampleCount = 0;
    private int targetSamples;

    public BallColorSurvey(int initialTarget) {
        this.targetSamples = initialTarget;
    }

    /**
     * Resets the survey and seeds it with the current "known" inventory.
     * This acts as a tie-breaker and adds stability.
     */
    public void reset(int newTarget, Character[] currentInventory) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) colorVotes[i][j] = 0;

            // Seed with current inventory: give the existing color 1 "legacy" vote
            char c = currentInventory[i];
            int colIdx = (c == 'P') ? 1 : (c == 'G' ? 2 : 0);
            colorVotes[i][colIdx] = 1;
        }
        this.sampleCount = 0;
        this.targetSamples = newTarget;
    }

    public void addSample(List<Character> currentRead) {
        for (int i = 0; i < 3; i++) {
            char c = currentRead.get(i);
            int colIdx = (c == 'P') ? 1 : (c == 'G' ? 2 : 0);
            colorVotes[i][colIdx]++;
        }
        sampleCount++;
    }

    public boolean isComplete() { return sampleCount >= targetSamples; }

    public Character[] getResult() {
        Character[] finalColors = new Character[3];
        for (int i = 0; i < 3; i++) {
            int maxIdx = 0;
            // If the sensor sees a new color consistently (e.g. 2 out of 3 times),
            // it will have 2 votes, beating the 1 "legacy" vote.
            if (colorVotes[i][1] > colorVotes[i][maxIdx]) maxIdx = 1;
            if (colorVotes[i][2] > colorVotes[i][maxIdx]) maxIdx = 2;

            finalColors[i] = (maxIdx == 1) ? 'P' : (maxIdx == 2 ? 'G' : 'N');
        }
        return finalColors;
    }
}
