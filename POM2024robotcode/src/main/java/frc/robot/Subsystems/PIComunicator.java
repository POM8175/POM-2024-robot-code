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
        
    HttpClient client;
    HttpResponse<String> response;

    ObjectMapper mapper = new ObjectMapper();
    JsonNode jsonNode;

    public PIComunicator()
    {
        try{
        request = HttpRequest.newBuilder()
            .uri(new URI("http://pi/getMyBloodyNote"))
            .GET()
            .build();

        client = HttpClient.newBuilder()
            .version(Version.HTTP_1_1)
            .followRedirects(Redirect.NORMAL)
            .connectTimeout(Duration.ofSeconds(20))
            .proxy(ProxySelector.of(new InetSocketAddress("proxy.example.com", 80)))
            .authenticator(Authenticator.getDefault())
            .build();

        response = client.send(request, BodyHandlers.ofString());
        }catch(IOException | URISyntaxException | InterruptedException e){
            e.printStackTrace();
        }
    }

    public int[] getNote()
    {
        int left = 0,right = 0,top = 0,buttom = 0;
        try{
            response = client.send(request, BodyHandlers.ofString());
            jsonNode = mapper.readTree(response.body());
            left = jsonNode.get("left").asInt();
            right = jsonNode.get("right").asInt();
            top = jsonNode.get("top").asInt();
            buttom = jsonNode.get("buttom").asInt();
        }catch(IOException | InterruptedException e){
            e.printStackTrace();
        }

        return new int[]{left,right,top,buttom};
    }

    public void close()
    {
       
    }
}