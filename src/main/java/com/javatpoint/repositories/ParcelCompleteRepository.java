package com.javatpoint.repositories;

import java.util.List;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.data.mongodb.core.MongoTemplate;
//import org.springframework.data.mongodb.repository.MongoRepository;
import org.springframework.stereotype.Repository;

// import org.springframework.data.mongodb.core.MongoTemplate;
import org.springframework.data.mongodb.core.query.Criteria;
import org.springframework.data.mongodb.core.query.Query;
import org.springframework.data.mongodb.core.query.Update;

import com.javatpoint.models.Parcel;
import com.javatpoint.models.ParcelComplete;
import com.javatpoint.models.ParcelComplete_a2a;
//import com.javatpoint.models.Status;
import com.javatpoint.models.TimestampTelemetry;
import com.javatpoint.models.User;
import com.javatpoint.models.TimestampAlarm;
import com.javatpoint.models.TimestampStatus;

@Repository
public class ParcelCompleteRepository{
        
    @Autowired
    private final MongoTemplate mongoTemplate;
    
    public ParcelCompleteRepository(MongoTemplate mongoTemplate) {
        this.mongoTemplate = mongoTemplate;
    }
    
    public ParcelComplete addTelemetry(String id, TimestampTelemetry newTimestampTelemetry) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().addToSet("telemetry", newTimestampTelemetry);
        return mongoTemplate.findAndModify(query, update, ParcelComplete.class);
    }

    public ParcelComplete addAlarm(String id, TimestampAlarm newTimestampAlarm) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().addToSet("alarm", newTimestampAlarm);
        return mongoTemplate.findAndModify(query, update, ParcelComplete.class);
    }

    public ParcelComplete addStatus(String id, TimestampStatus newTimestampStatus) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().addToSet("status", newTimestampStatus);
        return mongoTemplate.findAndModify(query, update, ParcelComplete.class);
    }
    
    public TimestampTelemetry getLastTelemetry(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id.toString()));
        ParcelComplete pC = mongoTemplate.findOne(query, ParcelComplete.class);
        List<TimestampTelemetry> telemetry = pC.getTelemetry(); // pobierz całą listę Telemetrii
        if(telemetry != null) 
            if(telemetry.size() != 0)
                return telemetry.get(telemetry.size() - 1); // wypisz ostatni wpis Telenetrii   //find, findOne, findAndModify

        return null;
    }

    public TimestampAlarm getLastAlarm(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id.toString()));
        ParcelComplete pC = mongoTemplate.findOne(query, ParcelComplete.class);
        List<TimestampAlarm> alarm = pC.getAlarm(); // pobierz całą listę Alarmów
        if(alarm != null) 
            if(alarm.size() != 0)
                return alarm.get(alarm.size() - 1); // wypisz ostatni wpis Alarmów   //find, findOne, FindAndModify

        return null;
    }

    public TimestampStatus getLastStatus(String pId) {
        Query query = new Query();
        query.addCriteria(Criteria.where("parcel.id").is(pId.toString()));
        ParcelComplete pC = mongoTemplate.findOne(query, ParcelComplete.class);
        List<TimestampStatus> status = pC.getStatus(); // pobierz całą listę Statusów
        if(status != null) 
            if(status.size() != 0)
                return status.get(status.size() - 1); // wypisz ostatni wpis Statusów   //find, findOne, FindAndModify

        return null;
    }

    public List<ParcelComplete> findSendersParcelCompletes(String login) {
        Query query = new Query();
        query.addCriteria(Criteria.where("parcel.sender.login").is(login));
        return mongoTemplate.find(query, ParcelComplete.class);
    }

    public ParcelComplete save(ParcelComplete parcelComplete) {
        return mongoTemplate.save(parcelComplete);
    }

    public ParcelComplete_a2a save(ParcelComplete_a2a parcelComplete_a2a) {
        return mongoTemplate.save(parcelComplete_a2a);
    }

    public ParcelComplete findOneByParcel(String parcelId) {
        Query query = new Query();
        query.addCriteria(Criteria.where("parcel.id").is(parcelId));
        return mongoTemplate.findOne(query, ParcelComplete.class);
    }

    public ParcelComplete_a2a findOneParcelComplete_a2aByParcel(String parcelId) {
        Query query = new Query();
        query.addCriteria(Criteria.where("parcel.id").is(parcelId));
        return mongoTemplate.findOne(query, ParcelComplete_a2a.class);
    }

    public ParcelComplete findOne(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        return mongoTemplate.findOne(query, ParcelComplete.class);
    }

    public Parcel setCourier(String id, User courier) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("courier", courier);
        return mongoTemplate.findAndModify(query, update, Parcel.class);
    }

    public Parcel setCourier(String id, String courierId) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("courier.id", courierId);
        return mongoTemplate.findAndModify(query, update, Parcel.class);
    }

    public int getCode(String id) {
        TimestampStatus tSS = getLastStatus(id);
        return tSS.getStatus().getCode();
    }

    public ParcelComplete_a2a findOneParcelComplete_a2a(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        return mongoTemplate.findOne(query, ParcelComplete_a2a.class);
    }

    /* code 1 */
    public ParcelComplete_a2a setA2a_code1_idle_close_box_ack(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code1_idle_close_box_ack.ack", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }

    /* code 3 */
    /* open */
    public ParcelComplete_a2a setA2a_code3_idle_open_courier_agree(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code3_idle_open_courier_agree", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }
    // public boolean getA2a_code3_idle_open_courier_agree(String id) {
    //     Query query = new Query();
    //     query.addCriteria(Criteria.where("id").is(id));
    //     ParcelComplete_a2a pCa2a = mongoTemplate.findOne(query, ParcelComplete_a2a.class);
    //     return pCa2a.a2a_code3_idle_open_courier_agree;
    // }
    public ParcelComplete_a2a setA2a_code3_idle_open_box_ack(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code3_idle_open_box_ack.ack", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }
    /* close and protect */
    public ParcelComplete_a2a setA2a_code3_protect_close_sender_agree(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code3_protect_close_sender_agree", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }
    public ParcelComplete_a2a setA2a_code3_protect_close_box_ack(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code3_protect_close_box_ack.ack", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }

    /* code 5 */
    /* go idle and open */
    public ParcelComplete_a2a setA2a_code5_idle_open_receiver_agree(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code5_idle_open_receiver_agree", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }
    public ParcelComplete_a2a setA2a_code5_idle_open_courier_agree(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code5_idle_open_courier_agree", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }
    public ParcelComplete_a2a setA2a_code5_idle_open_box_ack(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code5_idle_open_box_ack.ack", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }
    /* end */
    public ParcelComplete_a2a setA2a_code5_end_open_receiver_agree(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code5_end_open_receiver_agree", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }
    public ParcelComplete_a2a setA2a_code5_end_box_ack(String id) {
        Query query = new Query();
        query.addCriteria(Criteria.where("id").is(id));
        Update update = new Update().set("a2a_code5_end_box_ack.ack", true);
        return mongoTemplate.findAndModify(query, update, ParcelComplete_a2a.class);
    }

    
}