package frc.robot.Subsystems;

import java.io.IOException;
import java.net.Authenticator;
import java.net.InetSocketAddress;
import java.net.ProxySelector;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.http.HttpClient.Redirect;
import java.net.http.HttpClient.Version;
import java.net.http.HttpResponse.BodyHandlers;
import java.time.Duration;

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
            //TODO: put the correct URI
        request = HttpRequest.newBuilder()
            .uri(new URI("http://192.168.55.225:8001/getMyBloodyNote"))
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
                json = map.readTree(response.body());
                
                left = json.get("left").asInt();
                right = json.get("right").asInt();
                top = json.get("top").asInt();
                buttom = json.get("buttom").asInt();
            }catch(IOException | InterruptedException e){
            e.printStackTrace();
        }

        return new int[]{left,right,top,buttom};
    }
}