// app/api/users/login/route.ts
import { connect } from "@/dbConfig/dbConfig";
import User from "@/models/userModel";
import { NextResponse } from "next/server";
import bcryptjs from 'bcryptjs';
import jwt from 'jsonwebtoken';
import { serialize } from 'cookie';
import { withDemoCookies } from "@/helper/demoAuth";
import { getDemoAuth } from "@/helper/demoAuth";
import { isDemoMode } from "@/helper/demoMode";

const JWT_SECRET = process.env.JWT_SECRET as string;

export async function POST(request: Request) {
    if (isDemoMode()) {
        const response = NextResponse.json({
            message: "Demo login successful",
            success: true,
            user: getDemoAuth(),
        });
        return withDemoCookies(response);
    }

    try {
        await connect();
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

        const response = NextResponse.json({
            message: "Login successful",
            success: true,
            user: {
                id: user._id,
                username: user.username
            }
        });

        const cookieOptions = {
            httpOnly: true,
            maxAge: 3600,
            path: '/',
        };

        response.headers.append(
            'Set-Cookie',
            serialize('auth_token', token, cookieOptions)
        );
        response.headers.append(
            'Set-Cookie',
            serialize('username', user.username, cookieOptions)
        );

        return response;
    } catch (err: unknown) { // Use 'unknown' instead of 'any'
        console.error("Login error:", err);
        
        // Ensure err is an instance of Error before accessing 'message'
        const errorMessage = err instanceof Error ? err.message : "An unknown error occurred";
        
        return NextResponse.json({ error: errorMessage }, { status: 500 });
    }
}
