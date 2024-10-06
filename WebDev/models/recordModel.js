import mongoose from 'mongoose';

const recordSchema= new mongoose.Schema({
    Class: { 
        type: String, 
        required: true,  
    },
    Location:{
        type: String,
        required:true,
    },
    isBiodegradable:{
        type: Boolean,
        default: true,
    },
    Date:{
        type: String,
        default: true,
    },

})
const Record = mongoose.models.Record || mongoose.model("Record",recordSchema);
export default Record;