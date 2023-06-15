package com.javatpoint.models;

public class GeoCoordinates {
    private double longitude;
    //private double elevation;
    private double latitude;
    private String time;
    
    public GeoCoordinates () {
    
    }

    // Constructor
    public GeoCoordinates(double longitude, double latitude, String time) {
        this.longitude = longitude;
        this.latitude = latitude;
        this.time = time;
    }

    // Getters and Setters
    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }
    
    public String getTime() {
        return time;
    }

    public void setTime(String time) {
        this.time = time;
    }
}