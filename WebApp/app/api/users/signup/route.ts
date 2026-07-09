import { connect } from "@/dbConfig/dbConfig";
import User from "@/models/userModel";
import { NextRequest, NextResponse } from "next/server";
import bcryptjs from "bcryptjs";
import mongoose from "mongoose";
import { SAMPLE_WASTE_RECORDS } from "@/helper/seedWasteRecords";
import { withDemoCookies } from "@/helper/demoAuth";
import { getDemoAuth } from "@/helper/demoAuth";
import { isDemoMode } from "@/helper/demoMode";

export async function POST(request: NextRequest) {
    if (isDemoMode()) {
        const response = NextResponse.json({
            message: "Demo account ready",
            success: true,
            savedUser: getDemoAuth(),
        });
        return withDemoCookies(response);
    }

    try {
        await connect();
        const reqBody = await request.json();
        const { username, email, password } = reqBody;

        if (!username?.trim() || !email?.trim() || !password?.trim()) {
            return NextResponse.json({ error: "All fields are required" }, { status: 400 });
        }

        const trimmedUsername = username.trim();
        const trimmedEmail = email.trim().toLowerCase();

        const existingUser = await User.findOne({
            $or: [{ email: trimmedEmail }, { username: trimmedUsername }],
        });
        if (existingUser) {
            return NextResponse.json({ error: "User already exists" }, { status: 400 });
        }

        // Hash the password
        const salt = await bcryptjs.genSalt(10);
        const hashedPassword = await bcryptjs.hash(password, salt);

        // Create a new user
        const newUser = new User({
            username: trimmedUsername,
            email: trimmedEmail,
            password: hashedPassword,
            isVerified: true,
            isAdmin: false,
        });

        const savedUser = await newUser.save();

        const userWasteCollection = mongoose.connection.collection(
            `${trimmedUsername}_waste_records`
        );
        const existingRecords = await userWasteCollection.countDocuments();
        if (existingRecords === 0) {
            await userWasteCollection.insertMany(SAMPLE_WASTE_RECORDS);
        }

        const userKeys = mongoose.connection.collection(`${trimmedUsername}_keys`);
        await userKeys.createIndex({ _id: 1 });

        // Respond with success message
        return NextResponse.json({
            message: "User Registration Successful",
            success: true,
            savedUser,
        });
    } catch (error: unknown) {
        console.error("Error in signup process:", error); // Log the error

        // Ensure proper error handling
        const errorMessage = error instanceof Error ? error.message : "An unknown error occurred";

        return NextResponse.json({ error: errorMessage }, { status: 500 });
    }
}
