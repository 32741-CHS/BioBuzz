package org.firstinspires.ftc.testteamcode.utils;
import org.firstinspires.ftc.testteamcode.gameManager.gameManager.mainBot;
import org.firstinspires.ftc.testteamcode.gameManager.gameManager.gameStatus;
public static class varSync {
    /*
     public static boolean gameIsActive = false; //This value is synced with gameStatus.
    //If above value false, then gameStatus will be forced Inactive. Game status follows this value.
    //If above value true, allow other game states apart from Inactive.
    //This syncing is handled in utils/varsyncs
    public static gameStatus gameState = gameStatus.Inactive;
     */
    varSync()  {
        mainBot bot = new mainBot();
        //By importing bot here ^, we gain access to the static variables of said class.
        if(bot.gameIsActive == false)  {
            bot.gameState = gameStatus.Inactive;
        }
    }
}