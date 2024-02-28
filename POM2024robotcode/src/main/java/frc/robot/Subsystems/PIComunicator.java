package frc.robot.Subsystems;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIComunicator extends SubsystemBase
{
    HttpRequest request;
        HttpResponse<String> response;
        JsonNode json;
        ObjectMapper map = new ObjectMapper();

    public PIComunicator()
    {
        try{
            request = HttpRequest.newBuilder()
                .uri(new URI("http://10.81.75.14:8001/Note"))
                .GET()
                .build();


        }catch(URISyntaxException e){
            e.printStackTrace();
        }
    }

    public int[] getNote()
    {
        int left = 0,right = 0,top = 0,buttom = 0;
        try{
            response = HttpClient.newHttpClient().send(request,HttpResponse.BodyHandlers.ofString());
                System.out.println("response is: " + response.body());
                if(response.body() != "no notes detected")json = map.readTree(response.body());
                
                left = json.get("left").asInt();
                right = json.get("right").asInt();
                top = json.get("top").asInt();
                buttom = json.get("buttom").asInt();
            }catch(IOException | InterruptedException e){
            e.printStackTrace();
        }

        if((left==0)&&(right==0)&&(top==0)&&(buttom==0)) return null;
        return new int[]{left,right,top,buttom};
    }
}