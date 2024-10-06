// app/api/users/login/route.ts
import { connect } from "@/dbConfig/dbConfig";
import User from "@/models/userModel";
import { NextResponse } from "next/server";
import bcryptjs from 'bcryptjs';
import jwt from 'jsonwebtoken';
import { serialize } from 'cookie';

const JWT_SECRET = process.env.JWT_SECRET as string;

// Ensure the database is connected
await connect();

export async function POST(request: Request) {
    try {
        const reqBody = await request.json();
        const { email, password } = reqBody;

        const trimmedEmail = email.trim().toLowerCase();
        const trimmedPassword = password.trim();

        const user = await User.findOne({ email: trimmedEmail }).exec();

        if (!user) {
            return NextResponse.json({ error: "User does not exist" }, { status: 400 });
        }

        const isMatch = await bcryptjs.compare(trimmedPassword, user.password);
        if (!isMatch) {
            return NextResponse.json({ error: "Invalid credentials" }, { status: 400 });
        }

        // Generate JWT token
        const token = jwt.sign(
            { id: user._id, username: user.username }, 
            JWT_SECRET, 
            { expiresIn: '1h' }
        );

        // Set token in a cookie
        const cookie = serialize('auth_token', token, {
            httpOnly: true,
            maxAge: 3600,
            path: '/'
        });

        const response = NextResponse.json({
            message: "Login successful",
            success: true,
            user: {
                id: user._id,
                username: user.username
            }
        });
        
        // Attach the cookie to the response
        response.headers.append('Set-Cookie', cookie);
        return response;
    } catch (error: any) {
        console.error("Login error:", error);
        return NextResponse.json({ error: error.message }, { status: 500 });
    }
}
